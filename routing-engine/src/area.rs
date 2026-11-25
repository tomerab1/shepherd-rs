//! Areas to avoid (e.g. a region a user does not want to drive through).
//!
//! An area is a set of polygon rings combined with the even-odd rule (so holes work without
//! caring about ring roles). Each area gets its own customized metric in which roads inside
//! the area are heavily penalized: routes leave the area out whenever any alternative exists,
//! but a start or destination inside the area still works.

use crate::geo::Coord;
use crate::graph::Graph;
use anyhow::{bail, Context};
use osmpbf::{Element, ElementReader, RelMemberType};
use rayon::prelude::*;
use serde_json::{json, Value};
use std::collections::{HashMap, HashSet};
use std::path::Path;

/// Travel time inside an avoided area is multiplied by this for routing purposes.
pub const AVOID_FACTOR: f64 = 40.0;

#[derive(Clone, Debug)]
pub struct Area {
    /// Rings as (lon, lat) points, implicitly closed.
    pub rings: Vec<Vec<[f64; 2]>>,
    bbox: [f64; 4],
    /// Edges bucketed into horizontal bands: a horizontal ray only needs the edges of the
    /// band containing the point (~100x fewer tests for a detailed boundary).
    bands: Vec<Vec<[f64; 4]>>,
}

const BANDS: usize = 1024;

impl Area {
    pub fn new(rings: Vec<Vec<[f64; 2]>>) -> Area {
        let mut bbox = [f64::MAX, f64::MAX, f64::MIN, f64::MIN];
        for r in &rings {
            for p in r {
                bbox = [bbox[0].min(p[0]), bbox[1].min(p[1]), bbox[2].max(p[0]), bbox[3].max(p[1])];
            }
        }
        let mut bands = vec![Vec::new(); BANDS];
        let h = (bbox[3] - bbox[1]).max(1e-12);
        let band_of = |y: f64| (((y - bbox[1]) / h * BANDS as f64) as usize).min(BANDS - 1);
        for r in &rings {
            let n = r.len();
            for i in 0..n {
                let (a, b) = (r[i], r[(i + 1) % n]);
                let (lo, hi) = (band_of(a[1].min(b[1])), band_of(a[1].max(b[1])));
                for band in bands.iter_mut().take(hi + 1).skip(lo) {
                    band.push([a[0], a[1], b[0], b[1]]);
                }
            }
        }
        Area { rings, bbox, bands }
    }

    /// Reads Polygon / MultiPolygon geometries, Features or a FeatureCollection.
    pub fn from_geojson(path: &Path) -> anyhow::Result<Area> {
        let v: Value = serde_json::from_slice(&std::fs::read(path).with_context(|| format!("reading {}", path.display()))?)?;
        let mut rings = Vec::new();
        collect(&v, &mut rings)?;
        if rings.is_empty() {
            bail!("{} contains no polygons", path.display());
        }
        Ok(Area::new(rings))
    }

    pub fn to_geojson(&self, name: &str) -> Value {
        let polys: Vec<Value> = self
            .rings
            .iter()
            .map(|r| {
                let mut ring: Vec<[f64; 2]> = r.clone();
                if ring.first() != ring.last() {
                    ring.push(ring[0]);
                }
                json!([ring])
            })
            .collect();
        json!({
            "type": "Feature",
            "properties": { "name": name, "rule": "even-odd over all rings" },
            "geometry": { "type": "MultiPolygon", "coordinates": polys },
        })
    }

    /// Even-odd point-in-polygon over all rings.
    pub fn contains(&self, c: Coord) -> bool {
        let (x, y) = (c.lon(), c.lat());
        if x < self.bbox[0] || x > self.bbox[2] || y < self.bbox[1] || y > self.bbox[3] {
            return false;
        }
        let h = (self.bbox[3] - self.bbox[1]).max(1e-12);
        let band = (((y - self.bbox[1]) / h * BANDS as f64) as usize).min(BANDS - 1);
        let mut inside = false;
        for &[xi, yi, xj, yj] in &self.bands[band] {
            if (yi > y) != (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi {
                inside = !inside;
            }
        }
        inside
    }

    /// Bitset over segments: set if any geometry point of the segment lies inside the area.
    pub fn segments_inside(&self, g: &Graph) -> Vec<u64> {
        let n = g.segments.len();
        let flags: Vec<bool> = (0..n as u32)
            .into_par_iter()
            .map(|s| g.seg_geometry(s).iter().any(|&c| self.contains(c)))
            .collect();
        let mut bits = vec![0u64; n.div_ceil(64)];
        for (i, f) in flags.into_iter().enumerate() {
            if f {
                bits[i / 64] |= 1 << (i % 64);
            }
        }
        bits
    }
}

fn collect(v: &Value, rings: &mut Vec<Vec<[f64; 2]>>) -> anyhow::Result<()> {
    let ring_of = |r: &Value| -> anyhow::Result<Vec<[f64; 2]>> {
        r.as_array()
            .context("ring is not an array")?
            .iter()
            .map(|p| {
                let a = p.as_array().context("point is not an array")?;
                Ok([a[0].as_f64().context("bad lon")?, a[1].as_f64().context("bad lat")?])
            })
            .collect()
    };
    match v["type"].as_str() {
        Some("FeatureCollection") => {
            for f in v["features"].as_array().into_iter().flatten() {
                collect(f, rings)?;
            }
        }
        Some("Feature") => collect(&v["geometry"], rings)?,
        Some("Polygon") => {
            for r in v["coordinates"].as_array().into_iter().flatten() {
                rings.push(ring_of(r)?);
            }
        }
        Some("MultiPolygon") => {
            for poly in v["coordinates"].as_array().into_iter().flatten() {
                for r in poly.as_array().into_iter().flatten() {
                    rings.push(ring_of(r)?);
                }
            }
        }
        _ => {}
    }
    Ok(())
}

/// A relation found in a PBF whose tags match a search.
#[derive(Debug)]
pub struct FoundRelation {
    pub id: i64,
    pub tags: Vec<(String, String)>,
}

/// Lists boundary/multipolygon relations whose name tags contain `query` (case-insensitive).
pub fn find_relations(pbf: &str, query: &str) -> anyhow::Result<Vec<FoundRelation>> {
    let q = query.to_lowercase();
    let mut out = Vec::new();
    ElementReader::from_path(pbf)?.for_each(|el| {
        if let Element::Relation(r) = el {
            let tags: Vec<(String, String)> = r.tags().map(|(k, v)| (k.to_string(), v.to_string())).collect();
            let polygonal = tags.iter().any(|(k, v)| k == "type" && (v == "boundary" || v == "multipolygon"));
            let hit = tags.iter().any(|(k, v)| k.starts_with("name") && v.to_lowercase().contains(&q));
            if polygonal && hit {
                out.push(FoundRelation { id: r.id(), tags });
            }
        }
    })?;
    Ok(out)
}

/// Assembles the rings of relation `rel_id` from its member ways.
pub fn relation_area(pbf: &str, rel_id: i64) -> anyhow::Result<Area> {
    let mut members: Vec<i64> = Vec::new();
    ElementReader::from_path(pbf)?.for_each(|el| {
        if let Element::Relation(r) = el {
            if r.id() == rel_id {
                members = r
                    .members()
                    .filter(|m| m.member_type == RelMemberType::Way)
                    .map(|m| m.member_id)
                    .collect();
            }
        }
    })?;
    if members.is_empty() {
        bail!("relation {rel_id} not found or has no member ways");
    }
    let wanted: HashSet<i64> = members.iter().copied().collect();
    let mut ways: HashMap<i64, Vec<i64>> = HashMap::new();
    ElementReader::from_path(pbf)?.for_each(|el| {
        if let Element::Way(w) = el {
            if wanted.contains(&w.id()) {
                ways.insert(w.id(), w.refs().collect());
            }
        }
    })?;
    let needed: HashSet<i64> = ways.values().flatten().copied().collect();
    let mut nodes: HashMap<i64, [f64; 2]> = HashMap::new();
    ElementReader::from_path(pbf)?.for_each(|el| match el {
        Element::DenseNode(n) if needed.contains(&n.id()) => {
            nodes.insert(n.id(), [n.lon(), n.lat()]);
        }
        Element::Node(n) if needed.contains(&n.id()) => {
            nodes.insert(n.id(), [n.lon(), n.lat()]);
        }
        _ => {}
    })?;

    // Join ways end to end into closed rings.
    let mut pending: Vec<Vec<i64>> = members.iter().filter_map(|w| ways.get(w).cloned()).collect();
    let mut rings: Vec<Vec<i64>> = Vec::new();
    while let Some(mut ring) = pending.pop() {
        while ring.first() != ring.last() {
            let end = *ring.last().unwrap();
            let Some(i) = pending.iter().position(|w| w[0] == end || *w.last().unwrap() == end) else {
                break; // open ring: the extract clips the boundary; close it straight.
            };
            let mut w = pending.swap_remove(i);
            if w[0] != end {
                w.reverse();
            }
            ring.extend_from_slice(&w[1..]);
        }
        rings.push(ring);
    }
    let rings: Vec<Vec<[f64; 2]>> = rings
        .into_iter()
        .map(|r| r.iter().filter_map(|id| nodes.get(id).copied()).collect::<Vec<_>>())
        .filter(|r| r.len() >= 3)
        .collect();
    if rings.is_empty() {
        bail!("relation {rel_id} produced no rings");
    }
    Ok(Area::new(rings))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn even_odd_with_hole() {
        let outer = vec![[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]];
        let hole = vec![[4.0, 4.0], [6.0, 4.0], [6.0, 6.0], [4.0, 6.0]];
        let a = Area::new(vec![outer, hole]);
        assert!(a.contains(Coord::from_deg(2.0, 2.0)));
        assert!(!a.contains(Coord::from_deg(5.0, 5.0)));
        assert!(!a.contains(Coord::from_deg(20.0, 5.0)));
    }
}
