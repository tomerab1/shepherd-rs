//! Traffic-light waits, per signalised intersection.
//!
//! OSM maps a signalised intersection with several light nodes (one per approach,
//! carriageway or crossing), often only on some of its legs. Charging light *nodes* both
//! double-counts and misses intersections. Instead, at preprocessing every junction within
//! `INTERSECTION_M` of a light joins that light's intersection (`Graph::junction_cluster`), and:
//!
//! - **passing through** a junction of a signalised intersection costs one wait, unless the
//!   route is already inside that intersection (it came from another of its junctions);
//! - a light far from any junction is a mid-block crossing, charged when passed (once per
//!   cluster per segment).
//!
//! Waits are per intersection: the largest wait of its lights, so timing data uploaded for any
//! of an intersection's lights applies to it.

use crate::engine::{Overrides, Piece};
use crate::geo::Coord;
use crate::graph::Graph;
use crate::network::NONE_ID;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// How lights without timing data are treated.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum SignalMode {
    /// Lights without timing data get the profile's typical wait.
    #[default]
    Estimate,
    /// Only intersections with timing data (from the admin API) count; the rest are ignored.
    TimedOnly,
    /// Ignore traffic lights entirely.
    Off,
}

/// Mid-block lights (not part of an intersection) by segment.
pub struct SignalIndex {
    pub midblock: HashMap<u32, Vec<u32>>,
}

impl SignalIndex {
    pub fn new(g: &Graph) -> SignalIndex {
        let mut midblock: HashMap<u32, Vec<u32>> = HashMap::new();
        for (i, s) in g.signals.iter().enumerate() {
            if s.junction == NONE_ID && s.approach == 0 {
                midblock.entry(s.segment).or_default().push(i as u32);
            }
        }
        SignalIndex { midblock }
    }
}

/// Wait (deciseconds) per cluster id (cluster ids are light indices); 0 = ignored.
pub fn waits(g: &Graph, ov: &Overrides) -> Vec<u32> {
    let profile = g.profile();
    let mut w = vec![0u32; g.signals.len()];
    if ov.signal_mode == SignalMode::Off {
        return w;
    }
    for (i, s) in g.signals.iter().enumerate() {
        let secs = match ov.signal_delay.get(&(i as u32)) {
            Some(&d) => d as f64,
            None if ov.signal_mode == SignalMode::TimedOnly => continue,
            None => profile.signal_delay_s(s.junction != NONE_ID || s.approach != 0),
        };
        let ds = (secs.max(0.0) * 10.0).round() as u32;
        let c = s.cluster as usize;
        w[c] = w[c].max(ds);
    }
    w
}

#[inline]
fn junction_cluster(g: &Graph, j: u32) -> u32 {
    g.junction_cluster.get(j as usize).copied().unwrap_or(NONE_ID)
}

/// Active mid-block lights on element `e`'s segment, outside the intersection it starts at,
/// as (position along direction, cluster, coordinate), in travel order.
fn midblock(g: &Graph, idx: &SignalIndex, w: &[u32], e: u32) -> Vec<(f64, u32, Coord)> {
    let (seg, fwd) = g.elem_segment(e);
    let Some(list) = idx.midblock.get(&seg) else { return Vec::new() };
    let start = junction_cluster(g, g.elem_start_junction(e));
    let mut v: Vec<(f64, u32, Coord)> = list
        .iter()
        .map(|&s| &g.signals[s as usize])
        .filter(|s| w[s.cluster as usize] > 0 && s.cluster != start)
        .map(|s| {
            let f = s.fraction as f64;
            (if fwd { f } else { 1.0 - f }, s.cluster, s.coord)
        })
        .collect();
    v.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    v
}

/// Wait (ds) for mid-block lights on element `e`, once per cluster.
pub fn interior_wait(g: &Graph, idx: &SignalIndex, w: &[u32], e: u32) -> u32 {
    let mut seen: Vec<u32> = Vec::new();
    let mut total = 0;
    for (_, c, _) in midblock(g, idx, w, e) {
        if !seen.contains(&c) {
            seen.push(c);
            total += w[c as usize];
        }
    }
    total
}

/// The intersection element `e` is inside when it reaches its end junction.
fn inside(g: &Graph, idx: &SignalIndex, w: &[u32], e: u32) -> u32 {
    midblock(g, idx, w, e)
        .last()
        .map(|m| m.1)
        .unwrap_or_else(|| junction_cluster(g, g.elem_start_junction(e)))
}

/// Intersection whose wait is paid when passing the end junction of element `e`, if any.
pub fn end_junction_signal(g: &Graph, idx: &SignalIndex, w: &[u32], e: u32) -> Option<u32> {
    let c = junction_cluster(g, g.elem_end_junction(e));
    (c != NONE_ID && w[c as usize] > 0 && inside(g, idx, w, e) != c).then_some(c)
}

/// Lights a leg waits at (one per intersection or mid-block crossing), in travel order.
/// Intersections are placed at the junction passed.
pub fn route_lights(g: &Graph, idx: &SignalIndex, w: &[u32], pieces: &[Piece]) -> Vec<Coord> {
    let mut out: Vec<(u32, Coord)> = Vec::new();
    let mut push = |c: u32, at: Coord| {
        if out.last().is_none_or(|l| l.0 != c) {
            out.push((c, at));
        }
    };
    for (k, p) in pieces.iter().enumerate() {
        if k > 0 {
            let prev = &pieces[k - 1];
            let j = g.elem_start_junction(p.elem);
            let c = junction_cluster(g, j);
            // A leg that starts part-way along a segment never passed that segment's start
            // junction, so it is not inside that intersection yet.
            let started_inside = k == 1 && prev.from > 0.0 && c != NONE_ID && w[c as usize] > 0
                && midblock(g, idx, w, prev.elem).is_empty();
            if started_inside || end_junction_signal(g, idx, w, prev.elem).is_some() {
                push(c, g.junctions[j as usize]);
            }
        }
        for (pos, c, at) in midblock(g, idx, w, p.elem) {
            if pos >= p.from && pos <= p.to {
                push(c, at);
            }
        }
    }
    out.into_iter().map(|x| x.1).collect()
}

/// All signalised intersections and mid-block lights inside a bounding box
/// ([west, south, east, north]), one point each, for showing lights on a map.
pub fn lights_in(g: &Graph, w: &[u32], bbox: [f64; 4], limit: usize) -> Vec<Coord> {
    let inside = |c: Coord| c.lon() >= bbox[0] && c.lon() <= bbox[2] && c.lat() >= bbox[1] && c.lat() <= bbox[3];
    let mut seen: std::collections::HashSet<u32> = std::collections::HashSet::new();
    let mut out = Vec::new();
    for s in g.signals.iter() {
        if out.len() >= limit {
            break;
        }
        if w[s.cluster as usize] == 0 || !inside(s.coord) || !seen.insert(s.cluster) {
            continue;
        }
        out.push(s.coord);
    }
    out
}
