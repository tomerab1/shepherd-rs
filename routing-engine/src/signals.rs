//! Traffic-light waits.
//!
//! OSM maps a signalised intersection with several light nodes: one per approach (stop
//! lines just before the junction), per carriageway, or on the junction nodes themselves.
//! Charging every node would count one intersection two to four times. So lights are grouped
//! into intersections (`Signal::cluster`), stop-line lights only apply to traffic heading into
//! their junction, and a route pays **one wait per intersection it passes**:
//!
//! - inside a segment, applying lights are charged once per intersection, except the
//!   intersection the segment starts at (you just waited there);
//! - at a junction with a light, the wait is skipped if the segment you arrive on is already
//!   "inside" that intersection (you waited at its stop line, or came from another junction
//!   node of the same intersection).

use crate::engine::{Overrides, Piece};
use crate::graph::Graph;
use crate::network::{Signal, NONE_ID};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// How lights without timing data are treated.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum SignalMode {
    /// Lights without timing data get the profile's typical wait.
    #[default]
    Estimate,
    /// Only lights with timing data (from the admin API) count; the rest are ignored.
    TimedOnly,
    /// Ignore traffic lights entirely.
    Off,
}

/// Lookup from segments and junctions to their lights.
pub struct SignalIndex {
    pub by_segment: HashMap<u32, Vec<u32>>,
    pub by_junction: HashMap<u32, u32>,
}

impl SignalIndex {
    pub fn new(g: &Graph) -> SignalIndex {
        let mut by_segment: HashMap<u32, Vec<u32>> = HashMap::new();
        let mut by_junction = HashMap::new();
        for (i, s) in g.signals.iter().enumerate() {
            if s.junction != NONE_ID {
                by_junction.insert(s.junction, i as u32);
            } else {
                by_segment.entry(s.segment).or_default().push(i as u32);
            }
        }
        SignalIndex { by_segment, by_junction }
    }
}

/// Wait (deciseconds) per light under the current overrides; 0 means the light is ignored.
pub fn waits(g: &Graph, ov: &Overrides) -> Vec<u32> {
    let profile = g.profile();
    g.signals
        .iter()
        .enumerate()
        .map(|(i, s)| {
            if ov.signal_mode == SignalMode::Off {
                return 0;
            }
            let secs = match ov.signal_delay.get(&(i as u32)) {
                Some(&d) => d as f64,
                None if ov.signal_mode == SignalMode::TimedOnly => 0.0,
                None => profile.signal_delay_s(s.junction != NONE_ID || s.approach != 0),
            };
            (secs.max(0.0) * 10.0).round() as u32
        })
        .collect()
}

/// Cluster of the active light at the junction where element `e` starts, or `NONE_ID`.
#[inline]
fn start_cluster(g: &Graph, idx: &SignalIndex, w: &[u32], e: u32) -> u32 {
    match idx.by_junction.get(&g.elem_start_junction(e)) {
        Some(&s) if w[s as usize] > 0 => g.signals[s as usize].cluster,
        _ => NONE_ID,
    }
}

/// Active lights inside element `e`'s segment that apply in its direction, excluding the
/// intersection it starts at, as (position along direction, signal index).
fn interior(g: &Graph, idx: &SignalIndex, w: &[u32], e: u32) -> Vec<(f64, u32)> {
    let (seg, fwd) = g.elem_segment(e);
    let Some(list) = idx.by_segment.get(&seg) else { return Vec::new() };
    let skip = start_cluster(g, idx, w, e);
    let mut v: Vec<(f64, u32)> = list
        .iter()
        .copied()
        .filter(|&s| {
            let sig: &Signal = &g.signals[s as usize];
            w[s as usize] > 0 && sig.applies(fwd) && sig.cluster != skip
        })
        .map(|s| {
            let f = g.signals[s as usize].fraction as f64;
            (if fwd { f } else { 1.0 - f }, s)
        })
        .collect();
    v.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    v
}

/// Wait (ds) for lights inside element `e`, once per intersection.
pub fn interior_wait(g: &Graph, idx: &SignalIndex, w: &[u32], e: u32) -> u32 {
    let mut total = 0;
    let mut seen: Vec<u32> = Vec::new();
    for (_, s) in interior(g, idx, w, e) {
        let c = g.signals[s as usize].cluster;
        if !seen.contains(&c) {
            seen.push(c);
            total += w[s as usize];
        }
    }
    total
}

/// The intersection element `e` is already inside when it reaches its end junction:
/// its last applying stop-line light, else the light at its start junction.
fn inside_cluster(g: &Graph, idx: &SignalIndex, w: &[u32], e: u32) -> u32 {
    interior(g, idx, w, e)
        .last()
        .map(|&(_, s)| g.signals[s as usize].cluster)
        .unwrap_or_else(|| start_cluster(g, idx, w, e))
}

/// Signal at the junction where element `e` ends, if passing it costs a wait (it is active
/// and `e` is not already inside its intersection).
pub fn end_junction_signal(g: &Graph, idx: &SignalIndex, w: &[u32], e: u32) -> Option<u32> {
    let &s = idx.by_junction.get(&g.elem_end_junction(e))?;
    (w[s as usize] > 0 && inside_cluster(g, idx, w, e) != g.signals[s as usize].cluster).then_some(s)
}

/// Lights a leg passes and waits at, one per intersection, in travel order.
pub fn route_lights(g: &Graph, idx: &SignalIndex, w: &[u32], pieces: &[Piece]) -> Vec<u32> {
    let mut out: Vec<u32> = Vec::new();
    let push = |s: u32, out: &mut Vec<u32>| {
        let c = g.signals[s as usize].cluster;
        if out.last().is_none_or(|&l| g.signals[l as usize].cluster != c) {
            out.push(s);
        }
    };
    for (k, p) in pieces.iter().enumerate() {
        if k > 0 {
            if let Some(s) = end_junction_signal(g, idx, w, pieces[k - 1].elem) {
                push(s, &mut out);
            }
        }
        for (pos, s) in interior(g, idx, w, p.elem) {
            if pos >= p.from && pos <= p.to {
                push(s, &mut out);
            }
        }
    }
    out
}
