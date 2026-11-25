//! The compressed road network: junction nodes connected by segments.
//!
//! A segment is a stretch of one OSM way between two junctions (nodes shared by several ways,
//! way endpoints, or gaps in the data). Intermediate OSM nodes only survive as geometry points.

use crate::geo::{haversine, Coord};
use crate::osm::{profile::Profile, OsmData};
use bytemuck::{Pod, Zeroable};
use std::collections::HashMap;

/// Duration value meaning "not traversable in this direction".
pub const NO_DUR: u32 = u32::MAX;
pub const NO_NAME: u32 = u32::MAX;

pub const SEG_FORWARD: u8 = 1;
pub const SEG_BACKWARD: u8 = 2;
pub const SEG_ROUNDABOUT: u8 = 4;

/// One segment. `Pod` so the array is stored and mmap'd as-is.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Pod, Zeroable)]
pub struct Segment {
    /// Junction index at the start of the geometry.
    pub from: u32,
    /// Junction index at the end of the geometry.
    pub to: u32,
    /// First point in the geometry array; points `[geom_start, geom_start + geom_len)`.
    pub geom_start: u32,
    pub geom_len: u32,
    pub length_m: f32,
    /// Free-flow travel time in deciseconds for each direction, `NO_DUR` if not allowed.
    pub dur_fwd: u32,
    pub dur_bwd: u32,
    /// Index into the name table, `NO_NAME` if unnamed.
    pub name: u32,
    pub flags: u8,
    pub class: u8,
    pub _pad: [u8; 2],
}

impl Segment {
    #[inline]
    pub fn allows(&self, forward: bool) -> bool {
        self.flags & if forward { SEG_FORWARD } else { SEG_BACKWARD } != 0
    }
    #[inline]
    pub fn duration(&self, forward: bool) -> u32 {
        if forward {
            self.dur_fwd
        } else {
            self.dur_bwd
        }
    }
    #[inline]
    pub fn is_roundabout(&self) -> bool {
        self.flags & SEG_ROUNDABOUT != 0
    }
}

/// Interned strings in one contiguous buffer (an arena): `offsets[i]..offsets[i+1]`.
#[derive(Default, Debug, Clone)]
pub struct NameTable {
    pub bytes: Vec<u8>,
    pub offsets: Vec<u32>,
    index: HashMap<String, u32>,
}

impl NameTable {
    pub fn new() -> Self {
        Self {
            bytes: Vec::new(),
            offsets: vec![0],
            index: HashMap::new(),
        }
    }
    pub fn intern(&mut self, s: &str) -> u32 {
        if let Some(&i) = self.index.get(s) {
            return i;
        }
        let id = (self.offsets.len() - 1) as u32;
        self.bytes.extend_from_slice(s.as_bytes());
        self.offsets.push(self.bytes.len() as u32);
        self.index.insert(s.to_owned(), id);
        id
    }
}

/// A traffic light: either at a junction (`junction != NONE_ID`) or inside a segment at
/// `fraction` of its length (typically a pedestrian crossing).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Pod, Zeroable)]
pub struct Signal {
    pub osm_id: i64,
    pub coord: Coord,
    pub junction: u32,
    pub segment: u32,
    pub fraction: f32,
    pub _pad: u32,
}

pub const NONE_ID: u32 = u32::MAX;

#[derive(Debug, Default)]
pub struct Network {
    pub junctions: Vec<Coord>,
    pub junction_osm: Vec<i64>,
    pub junction_signal: Vec<bool>,
    pub segments: Vec<Segment>,
    /// OSM way id of each segment (used for restrictions and traffic updates).
    pub segment_way: Vec<i64>,
    pub geometry: Vec<Coord>,
    pub names: NameTable,
    /// Traffic lights. Their delay is applied by the metric, so it can change at runtime.
    pub signals: Vec<Signal>,
}

#[inline]
fn deciseconds(length_m: f64, speed_kmh: f32, extra_s: f64) -> u32 {
    let s = length_m / (speed_kmh as f64 / 3.6) + extra_s;
    ((s * 10.0).ceil() as u32).max(1)
}

impl Network {
    pub fn build(osm: &OsmData, _profile: Profile) -> Network {
        // Split each way into runs of consecutive nodes that have coordinates.
        let mut runs: Vec<(usize, Vec<i64>)> = Vec::with_capacity(osm.ways.len());
        for (wi, way) in osm.ways.iter().enumerate() {
            let mut run: Vec<i64> = Vec::new();
            for &r in &way.refs {
                if osm.node(r).is_none() {
                    if run.len() >= 2 {
                        runs.push((wi, std::mem::take(&mut run)));
                    }
                    run.clear();
                    continue;
                }
                if run.last() != Some(&r) {
                    run.push(r);
                }
            }
            if run.len() >= 2 {
                runs.push((wi, run));
            }
        }

        // A node is a junction if it is used more than once or ends a run.
        let mut uses: HashMap<i64, u32> = HashMap::with_capacity(osm.nodes.len());
        for (_, run) in &runs {
            for &r in run {
                *uses.entry(r).or_insert(0) += 1;
            }
            *uses.get_mut(&run[0]).unwrap() += 2;
            *uses.get_mut(run.last().unwrap()).unwrap() += 2;
        }
        // Restriction via nodes must be junctions so the turn exists in the graph.
        for r in &osm.restrictions {
            if let Some(u) = r.via_node().and_then(|n| uses.get_mut(&n)) {
                *u += 2;
            }
        }

        let mut net = Network {
            names: NameTable::new(),
            ..Default::default()
        };
        let mut junction_of: HashMap<i64, u32> = HashMap::new();
        let mut junction = |osm_id: i64, net: &mut Network| -> u32 {
            *junction_of.entry(osm_id).or_insert_with(|| {
                net.junctions.push(osm.node(osm_id).unwrap());
                net.junction_osm.push(osm_id);
                net.junction_signal.push(osm.is_signal(osm_id));
                (net.junctions.len() - 1) as u32
            })
        };

        for (wi, run) in &runs {
            let way = &osm.ways[*wi];
            let p = &way.profile;
            let name = p.name.as_deref().map(|n| net.names.intern(n)).unwrap_or(NO_NAME);
            let mut flags = 0u8;
            if p.forward {
                flags |= SEG_FORWARD;
            }
            if p.backward {
                flags |= SEG_BACKWARD;
            }
            if p.roundabout {
                flags |= SEG_ROUNDABOUT;
            }

            let mut start = 0usize;
            for i in 1..run.len() {
                let is_junction = uses[&run[i]] > 1;
                if !is_junction && i != run.len() - 1 {
                    continue;
                }
                let piece = &run[start..=i];
                let from = junction(piece[0], &mut net);
                let to = junction(piece[piece.len() - 1], &mut net);
                let geom_start = net.geometry.len() as u32;
                let mut length = 0.0;
                let mut interior: Vec<(i64, Coord, f64)> = Vec::new();
                let mut prev = None;
                for (k, &id) in piece.iter().enumerate() {
                    let c = osm.node(id).unwrap();
                    if let Some(pc) = prev {
                        length += haversine(pc, c);
                    }
                    if k > 0 && k + 1 < piece.len() && osm.is_signal(id) {
                        interior.push((id, c, length));
                    }
                    net.geometry.push(c);
                    prev = Some(c);
                }
                let seg_id = net.segments.len() as u32;
                for (osm_id, coord, at) in interior {
                    net.signals.push(Signal {
                        osm_id,
                        coord,
                        junction: NONE_ID,
                        segment: seg_id,
                        fraction: if length > 0.0 { (at / length) as f32 } else { 0.0 },
                        _pad: 0,
                    });
                }
                let extra = 0.0;
                net.segments.push(Segment {
                    from,
                    to,
                    geom_start,
                    geom_len: piece.len() as u32,
                    length_m: length as f32,
                    dur_fwd: if p.forward { deciseconds(length, p.speed_fwd, extra) } else { NO_DUR },
                    dur_bwd: if p.backward { deciseconds(length, p.speed_bwd, extra) } else { NO_DUR },
                    name,
                    flags,
                    class: p.class as u8,
                    _pad: [0; 2],
                });
                net.segment_way.push(way.id);
                start = i;
            }
        }
        for j in 0..net.junctions.len() {
            if net.junction_signal[j] {
                net.signals.push(Signal {
                    osm_id: net.junction_osm[j],
                    coord: net.junctions[j],
                    junction: j as u32,
                    segment: NONE_ID,
                    fraction: 0.0,
                    _pad: 0,
                });
            }
        }
        net
    }

    #[inline]
    pub fn geometry_of(&self, seg: &Segment) -> &[Coord] {
        &self.geometry[seg.geom_start as usize..(seg.geom_start + seg.geom_len) as usize]
    }
}
