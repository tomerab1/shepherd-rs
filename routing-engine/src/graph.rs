//! The complete preprocessed routing graph, as stored in (and mapped from) the graph file.

use crate::cch::Topology;
use crate::ebg::{dseg_forward, dseg_seg};
use crate::geo::{Coord, LocalProjection};
use crate::network::{Segment, NO_NAME};
use crate::store::{Arr, Reader, Writer};
use serde::{Deserialize, Serialize};
use std::path::Path;

/// Small metadata block stored as JSON inside the graph file.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Meta {
    pub source: String,
    /// Routing profile this graph was built for ("driving" or "walking").
    #[serde(default = "default_profile")]
    pub profile: String,
    /// Junction-based routing graph (no turn restrictions/costs); otherwise edge-based.
    #[serde(default)]
    pub node_based: bool,
    pub built_at_unix: u64,
    pub ref_lat: f64,
    pub grid_origin: [f64; 2],
    pub grid_cell_m: f64,
    pub grid_nx: u32,
    pub grid_ny: u32,
    pub bbox: [f64; 4],
    pub num_junctions: usize,
    pub num_segments: usize,
    pub num_ebg_nodes: usize,
    pub num_ebg_arcs: usize,
    pub num_cch_edges: usize,
    #[serde(default)]
    pub num_signals: usize,
    pub elimination_tree_depth: usize,
    pub restrictions_applied: usize,
    pub restrictions_skipped: usize,
}

fn default_profile() -> String {
    "driving".into()
}

pub struct Graph {
    pub meta: Meta,
    pub junctions: Arr<Coord>,
    pub segments: Arr<Segment>,
    pub geometry: Arr<Coord>,
    pub name_bytes: Arr<u8>,
    pub name_offsets: Arr<u32>,
    pub segment_way: Arr<i64>,
    /// Segment ids sorted by OSM way id (for way-based traffic updates).
    pub way_order: Arr<u32>,
    /// Per segment: EBG node of [forward, backward] (`NO_NODE` if not allowed).
    pub seg_elems: Arr<u32>,
    pub elem_dseg: Arr<u32>,
    pub elem_flags: Arr<u8>,
    pub first_out: Arr<u32>,
    pub head: Arr<u32>,
    /// Turn cost per arc (edge-based graphs only; empty for node-based ones).
    pub turn: Arr<u16>,
    /// Network junction -> routing node (node-based graphs only).
    pub junction_node: Arr<u32>,
    pub topo: Topology,
    pub arc_to_cch: Arr<u32>,
    pub grid_first: Arr<u32>,
    pub grid_items: Arr<u32>,
    pub signals: Arr<crate::network::Signal>,
}

impl Graph {
    pub fn name(&self, id: u32) -> &str {
        if id == NO_NAME {
            return "";
        }
        let (a, b) = (self.name_offsets[id as usize], self.name_offsets[id as usize + 1]);
        std::str::from_utf8(&self.name_bytes[a as usize..b as usize]).unwrap_or("")
    }

    #[inline]
    pub fn seg_geometry(&self, s: u32) -> &[Coord] {
        let seg = &self.segments[s as usize];
        &self.geometry[seg.geom_start as usize..(seg.geom_start + seg.geom_len) as usize]
    }

    /// Segment owning geometry point `p` (segments' geometry ranges are contiguous and sorted).
    #[inline]
    pub fn segment_of_point(&self, p: u32) -> u32 {
        (self.segments.partition_point(|s| s.geom_start <= p) - 1) as u32
    }

    #[inline]
    pub fn elem_segment(&self, node: u32) -> (u32, bool) {
        let d = self.elem_dseg[node as usize];
        (dseg_seg(d), dseg_forward(d))
    }

    #[inline]
    pub fn seg_elem(&self, seg: u32, forward: bool) -> u32 {
        self.seg_elems[seg as usize * 2 + (!forward) as usize]
    }

    pub fn profile(&self) -> crate::osm::profile::Profile {
        crate::osm::profile::Profile::parse(&self.meta.profile).unwrap_or(crate::osm::profile::Profile::Car)
    }

    /// Junction where element `n` (a directed segment) ends.
    #[inline]
    pub fn elem_end_junction(&self, n: u32) -> u32 {
        let (s, fwd) = self.elem_segment(n);
        let seg = &self.segments[s as usize];
        if fwd {
            seg.to
        } else {
            seg.from
        }
    }

    /// Number of ways to continue at the end of element `e` (used to decide whether a
    /// junction offers a choice worth an instruction).
    #[inline]
    pub fn elem_choices(&self, e: u32) -> usize {
        let v = if self.meta.node_based { self.head[e as usize] } else { e };
        (self.first_out[v as usize + 1] - self.first_out[v as usize]) as usize
    }

    /// Junction where element `n` (a directed segment) starts.
    #[inline]
    pub fn elem_start_junction(&self, n: u32) -> u32 {
        let (s, fwd) = self.elem_segment(n);
        let seg = &self.segments[s as usize];
        if fwd {
            seg.from
        } else {
            seg.to
        }
    }

    pub fn projection(&self) -> LocalProjection {
        LocalProjection::new(self.meta.ref_lat)
    }

    /// Segments belonging to an OSM way.
    pub fn segments_of_way(&self, way: i64) -> impl Iterator<Item = u32> + '_ {
        let start = self
            .way_order
            .partition_point(|&s| self.segment_way[s as usize] < way);
        self.way_order[start..]
            .iter()
            .copied()
            .take_while(move |&s| self.segment_way[s as usize] == way)
    }

    pub fn save(&self, path: &Path) -> anyhow::Result<()> {
        let tmp = path.with_extension("tmp");
        let mut w = Writer::create(&tmp)?;
        w.add("meta", serde_json::to_vec(&self.meta)?.as_slice())?;
        w.add("junctions", &self.junctions[..])?;
        w.add("segments", &self.segments[..])?;
        w.add("geometry", &self.geometry[..])?;
        w.add("name_bytes", &self.name_bytes[..])?;
        w.add("name_offsets", &self.name_offsets[..])?;
        w.add("segment_way", &self.segment_way[..])?;
        w.add("way_order", &self.way_order[..])?;
        w.add("seg_elems", &self.seg_elems[..])?;
        w.add("elem_dseg", &self.elem_dseg[..])?;
        w.add("elem_flags", &self.elem_flags[..])?;
        w.add("first_out", &self.first_out[..])?;
        w.add("head", &self.head[..])?;
        w.add("turn", &self.turn[..])?;
        w.add("junction_node", &self.junction_node[..])?;
        w.add("cch_first_up", &self.topo.first_up[..])?;
        w.add("cch_up_head", &self.topo.up_head[..])?;
        w.add("cch_first_down", &self.topo.first_down[..])?;
        w.add("cch_down_tail", &self.topo.down_tail[..])?;
        w.add("cch_down_edge", &self.topo.down_edge[..])?;
        w.add("cch_parent", &self.topo.parent[..])?;
        w.add("cch_level_first", &self.topo.level_first[..])?;
        w.add("cch_level_nodes", &self.topo.level_nodes[..])?;
        w.add("arc_to_cch", &self.arc_to_cch[..])?;
        w.add("grid_first", &self.grid_first[..])?;
        w.add("grid_items", &self.grid_items[..])?;
        w.add("signals", &self.signals[..])?;
        w.finish()?;
        // Atomic replace so a running server can keep using the old mapping.
        std::fs::rename(&tmp, path)?;
        Ok(())
    }

    pub fn load(path: &Path) -> anyhow::Result<Graph> {
        let r = Reader::open(path)?;
        let meta: Meta = serde_json::from_slice(r.bytes("meta")?)?;
        Ok(Graph {
            meta,
            junctions: r.get("junctions")?,
            segments: r.get("segments")?,
            geometry: r.get("geometry")?,
            name_bytes: r.get("name_bytes")?,
            name_offsets: r.get("name_offsets")?,
            segment_way: r.get("segment_way")?,
            way_order: r.get("way_order")?,
            seg_elems: r.get("seg_elems")?,
            elem_dseg: r.get("elem_dseg")?,
            elem_flags: r.get("elem_flags")?,
            first_out: r.get("first_out")?,
            head: r.get("head")?,
            turn: r.get("turn")?,
            junction_node: r.get("junction_node")?,
            topo: Topology {
                first_up: r.get("cch_first_up")?,
                up_head: r.get("cch_up_head")?,
                first_down: r.get("cch_first_down")?,
                down_tail: r.get("cch_down_tail")?,
                down_edge: r.get("cch_down_edge")?,
                parent: r.get("cch_parent")?,
                level_first: r.get("cch_level_first")?,
                level_nodes: r.get("cch_level_nodes")?,
            },
            arc_to_cch: r.get("arc_to_cch")?,
            grid_first: r.get("grid_first")?,
            grid_items: r.get("grid_items")?,
            signals: r.get("signals")?,
        })
    }
}
