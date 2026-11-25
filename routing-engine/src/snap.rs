//! Snapping coordinates to the road network with a flat uniform grid.
//!
//! The grid is two `u32` arrays (cell offsets and geometry-piece ids) stored in the graph file,
//! so it costs ~4 bytes per road piece and needs no build step at server start, unlike an
//! in-memory R-tree.

use crate::ebg::NODE_ROUTABLE;
use crate::geo::{fast_dist, haversine, lerp, project_on_segment, Coord, LocalProjection};
use crate::graph::Graph;
use crate::network::Segment;

pub struct GridBuild {
    pub origin: [f64; 2],
    pub cell_m: f64,
    pub nx: u32,
    pub ny: u32,
    pub first: Vec<u32>,
    pub items: Vec<u32>,
}

/// Builds the grid over all geometry pieces of segments with a routable direction.
pub fn build_grid(
    segments: &[Segment],
    geometry: &[Coord],
    snappable: impl Fn(usize) -> bool,
    proj: &LocalProjection,
) -> GridBuild {
    let mut pieces: Vec<u32> = Vec::new();
    let (mut minx, mut miny, mut maxx, mut maxy) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
    for (s, seg) in segments.iter().enumerate() {
        if !snappable(s) {
            continue;
        }
        for p in seg.geom_start..seg.geom_start + seg.geom_len - 1 {
            pieces.push(p);
        }
        for p in seg.geom_start..seg.geom_start + seg.geom_len {
            let (x, y) = proj.project(geometry[p as usize]);
            minx = minx.min(x);
            miny = miny.min(y);
            maxx = maxx.max(x);
            maxy = maxy.max(y);
        }
    }
    if pieces.is_empty() {
        return GridBuild {
            origin: [0.0, 0.0],
            cell_m: 1000.0,
            nx: 1,
            ny: 1,
            first: vec![0, 0],
            items: vec![],
        };
    }
    let area = ((maxx - minx).max(1.0)) * ((maxy - miny).max(1.0));
    // About one piece per cell on average, but cells no smaller than 50 m.
    let cell_m = (area / pieces.len() as f64).sqrt().max(50.0);
    let nx = (((maxx - minx) / cell_m).floor() as u32 + 1).max(1);
    let ny = (((maxy - miny) / cell_m).floor() as u32 + 1).max(1);
    let cell = |x: f64, y: f64| -> (u32, u32) {
        (
            (((x - minx) / cell_m).floor() as i64).clamp(0, nx as i64 - 1) as u32,
            (((y - miny) / cell_m).floor() as i64).clamp(0, ny as i64 - 1) as u32,
        )
    };

    let mut pairs: Vec<(u32, u32)> = Vec::with_capacity(pieces.len() * 2);
    for &p in &pieces {
        let a = proj.project(geometry[p as usize]);
        let b = proj.project(geometry[p as usize + 1]);
        let (cx0, cy0) = cell(a.0.min(b.0), a.1.min(b.1));
        let (cx1, cy1) = cell(a.0.max(b.0), a.1.max(b.1));
        for cy in cy0..=cy1 {
            for cx in cx0..=cx1 {
                pairs.push((cy * nx + cx, p));
            }
        }
    }
    pairs.sort_unstable();
    let ncell = (nx * ny) as usize;
    let mut first = vec![0u32; ncell + 1];
    for &(c, _) in &pairs {
        first[c as usize + 1] += 1;
    }
    for i in 0..ncell {
        first[i + 1] += first[i];
    }
    GridBuild {
        origin: [minx, miny],
        cell_m,
        nx,
        ny,
        first,
        items: pairs.into_iter().map(|p| p.1).collect(),
    }
}

/// A coordinate snapped onto a segment.
#[derive(Clone, Copy, Debug)]
pub struct Snap {
    pub segment: u32,
    /// Position along the segment's forward geometry, 0..=1 (by length).
    pub fraction: f64,
    pub point: Coord,
    pub distance_m: f64,
    /// Forward bearing of the segment at the snapped point (degrees).
    pub bearing: f64,
}

impl Graph {
    /// Nearest routable road position within `max_m` meters.
    pub fn snap(&self, c: Coord, max_m: f64) -> Option<Snap> {
        let proj = self.projection();
        let p = proj.project(c);
        let m = &self.meta;
        let cx = ((p.0 - m.grid_origin[0]) / m.grid_cell_m).floor() as i64;
        let cy = ((p.1 - m.grid_origin[1]) / m.grid_cell_m).floor() as i64;
        let max_ring = (max_m / m.grid_cell_m).ceil() as i64 + 1;

        let mut best: Option<(f64, u32, f64)> = None; // (dist2, piece, t)
        for r in 0..=max_ring {
            for y in cy - r..=cy + r {
                if y < 0 || y >= m.grid_ny as i64 {
                    continue;
                }
                for x in cx - r..=cx + r {
                    if x < 0 || x >= m.grid_nx as i64 {
                        continue;
                    }
                    // Only the ring's border cells are new.
                    if r > 0 && y != cy - r && y != cy + r && x != cx - r && x != cx + r {
                        continue;
                    }
                    let cell = (y as u32 * m.grid_nx + x as u32) as usize;
                    for &piece in &self.grid_items[self.grid_first[cell] as usize..self.grid_first[cell + 1] as usize] {
                        let a = proj.project(self.geometry[piece as usize]);
                        let b = proj.project(self.geometry[piece as usize + 1]);
                        let (t, d2) = project_on_segment(p, a, b);
                        if best.is_none_or(|(bd, _, _)| d2 < bd) {
                            best = Some((d2, piece, t));
                        }
                    }
                }
            }
            // Everything in later rings is at least r * cell away.
            if let Some((d2, _, _)) = best {
                if d2.sqrt() <= r as f64 * m.grid_cell_m {
                    break;
                }
            }
        }
        let (d2, piece, t) = best?;
        if d2.sqrt() > max_m {
            return None;
        }
        let seg_id = self.segment_of_point(piece);
        let seg = &self.segments[seg_id as usize];
        let geom = self.seg_geometry(seg_id);
        let k = (piece - seg.geom_start) as usize;
        let mut along = 0.0;
        for w in geom[..=k].windows(2) {
            along += fast_dist(w[0], w[1]);
        }
        along += t * fast_dist(geom[k], geom[k + 1]);
        let total = geom.windows(2).map(|w| fast_dist(w[0], w[1])).sum::<f64>();
        let point = lerp(geom[k], geom[k + 1], t);
        Some(Snap {
            segment: seg_id,
            fraction: if total > 0.0 { (along / total).clamp(0.0, 1.0) } else { 0.0 },
            point,
            distance_m: haversine(c, point),
            bearing: crate::geo::bearing(geom[k], geom[k + 1]),
        })
    }

    /// Whether a segment has at least one direction in the routable component.
    pub fn segment_routable(&self, s: u32) -> bool {
        [true, false].iter().any(|&f| {
            let n = self.seg_elem(s, f);
            n != crate::ebg::NO_NODE && self.elem_flags[n as usize] & NODE_ROUTABLE != 0
        })
    }

    /// All segments with geometry within `radius_m` of `c` (used for area closures).
    pub fn segments_near(&self, c: Coord, radius_m: f64) -> Vec<u32> {
        let proj = self.projection();
        let p = proj.project(c);
        let m = &self.meta;
        let r = (radius_m / m.grid_cell_m).ceil() as i64;
        let cx = ((p.0 - m.grid_origin[0]) / m.grid_cell_m).floor() as i64;
        let cy = ((p.1 - m.grid_origin[1]) / m.grid_cell_m).floor() as i64;
        let mut out = Vec::new();
        for y in (cy - r).max(0)..=(cy + r).min(m.grid_ny as i64 - 1) {
            for x in (cx - r).max(0)..=(cx + r).min(m.grid_nx as i64 - 1) {
                let cell = (y as u32 * m.grid_nx + x as u32) as usize;
                for &piece in &self.grid_items[self.grid_first[cell] as usize..self.grid_first[cell + 1] as usize] {
                    let a = proj.project(self.geometry[piece as usize]);
                    let b = proj.project(self.geometry[piece as usize + 1]);
                    let (_, d2) = project_on_segment(p, a, b);
                    if d2 <= radius_m * radius_m {
                        out.push(self.segment_of_point(piece));
                    }
                }
            }
        }
        out.sort_unstable();
        out.dedup();
        out
    }
}
