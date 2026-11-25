//! The routing engine: graph + current metric + live overrides + query worker buffers.
//!
//! The metric (all weights) is immutable once built and published through an `ArcSwap`.
//! Traffic updates and closures build a new metric in the background (customization is
//! parallel and takes well under a second for a country) and swap it in atomically; queries
//! in flight keep using the metric they started with.

use crate::cch::customize::{customize, Metric};
use crate::cch::query::{query, InputArcs, QueryBuffers};
use crate::cch::INF;
use crate::ebg::{dseg_seg, NODE_ROUTABLE, NO_NODE};
use crate::geo::Coord;
use crate::graph::Graph;
use crate::snap::Snap;
use arc_swap::ArcSwap;
use parking_lot::Mutex;
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::sync::{Arc, OnceLock};
use std::time::Instant;

/// How far from a road a waypoint may be.
pub const MAX_SNAP_M: f64 = 2_000.0;

/// Live changes on top of the free-flow profile. Keyed by segment so they survive any
/// number of updates and can be listed back.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Overrides {
    /// Segment -> speed in km/h for [forward, backward]; `None` keeps the profile speed.
    pub speed: BTreeMap<u32, [Option<f32>; 2]>,
    /// Segment -> multiplier on travel time for [forward, backward] (e.g. 2.0 = twice as slow).
    pub factor: BTreeMap<u32, [Option<f32>; 2]>,
    /// Fully closed segments (both directions).
    pub closed: BTreeSet<u32>,
    /// Traffic light index -> expected wait in seconds (from signal timing data).
    pub signal_delay: BTreeMap<u32, f32>,
    /// How lights without timing data are treated.
    #[serde(default)]
    pub signal_mode: crate::signals::SignalMode,
    /// Live traffic feed: segment -> travel-time factor [forward, backward]. Replaced on
    /// every poll; combines with manual `factor`.
    #[serde(default)]
    pub feed_factor: BTreeMap<u32, [Option<f32>; 2]>,
}

impl Overrides {
    pub fn is_empty(&self) -> bool {
        self.speed.is_empty()
            && self.factor.is_empty()
            && self.closed.is_empty()
            && self.signal_delay.is_empty()
            && self.feed_factor.is_empty()
            && self.signal_mode == crate::signals::SignalMode::Estimate
    }
}

/// One published metric.
pub struct MetricState {
    pub metric: Metric,
    /// Real travel time of each EBG node (directed segment) in deciseconds (with traffic).
    /// Shared between the base metric and the area metrics derived from it.
    pub elem_dur: Arc<Vec<u32>>,
    /// Routing weight of each EBG node: `elem_dur`, plus penalties such as avoided areas.
    pub elem_w: Arc<Vec<u32>>,
    /// Expected traffic-light wait when entering each EBG node from its start junction
    /// (node-based graphs: when passing the arc's end junction).
    pub elem_entry: Arc<Vec<u32>>,
    /// Edge-based graphs: arcs whose tail is already inside the next junction's intersection
    /// (they skip its light wait).
    pub arc_skip: Arc<Vec<u64>>,
    /// Wait per traffic light in deciseconds under this metric (0 = ignored).
    pub signal_waits: Arc<Vec<u32>>,
    pub version: u64,
    pub customize_ms: f64,
}

/// The free-flow+traffic metric plus one penalized metric per avoidable area. Area metrics
/// are customized lazily on first use (and again after each traffic update), so areas nobody
/// asks for cost no memory.
pub struct Metrics {
    pub base: Arc<MetricState>,
    pub avoid: Vec<OnceLock<Arc<MetricState>>>,
    pub overrides: Arc<Overrides>,
}

/// An avoidable area resolved to the graph's segments.
pub struct AvoidArea {
    pub name: String,
    pub segments: Vec<u64>,
    pub count: usize,
}

pub struct Engine {
    pub graph: Graph,
    pub areas: Vec<AvoidArea>,
    pub signal_index: crate::signals::SignalIndex,
    metric: ArcSwap<Metrics>,
    overrides: Mutex<Overrides>,
    /// Serializes metric rebuilds so updates apply in order.
    rebuild: Mutex<()>,
    buffers: Mutex<Vec<QueryBuffers>>,
}

#[derive(Debug, thiserror::Error)]
pub enum RouteError {
    #[error("no road found near waypoint {0}")]
    NoSegment(usize),
    #[error("no route found between waypoints {0} and {1}")]
    NoRoute(usize, usize),
    #[error("{0}")]
    InvalidQuery(String),
}

/// One piece of a leg: a directed segment traversed from `from` to `to` (fractions along the
/// driving direction).
#[derive(Clone, Copy, Debug)]
pub struct Piece {
    /// Routing element of this directed segment (EBG node, or arc in node-based graphs).
    pub elem: u32,
    pub segment: u32,
    pub forward: bool,
    pub from: f64,
    pub to: f64,
    /// Turn cost paid when entering this piece (deciseconds); 0 for the first piece.
    pub turn_ds: u32,
}

pub struct LegPath {
    pub pieces: Vec<Piece>,
    /// Total weight (= travel time) in deciseconds.
    pub weight_ds: u64,
}

impl Engine {
    pub fn new(graph: Graph) -> Engine {
        Engine::with_areas(graph, Vec::new())
    }

    /// Builds an engine with avoidable areas (each gets its own customized metric).
    pub fn with_areas(graph: Graph, areas: Vec<(String, crate::area::Area)>) -> Engine {
        let areas: Vec<AvoidArea> = areas
            .into_iter()
            .map(|(name, a)| {
                let segments = a.segments_inside(&graph);
                let count = segments.iter().map(|w| w.count_ones() as usize).sum();
                tracing::info!(area = %name, segments = count, "resolved avoid area");
                AvoidArea { name, segments, count }
            })
            .collect();
        let ov = Overrides::default();
        let metrics = build_metrics(&graph, &areas, &ov, 0);
        tracing::info!("customized free-flow metric in {:.0} ms", metrics.base.customize_ms);
        let signal_index = crate::signals::SignalIndex::new(&graph);
        Engine {
            graph,
            areas,
            signal_index,
            metric: ArcSwap::from_pointee(metrics),
            overrides: Mutex::new(ov),
            rebuild: Mutex::new(()),
            buffers: Mutex::new(Vec::new()),
        }
    }

    /// The current base metric (traffic-aware, no area penalties).
    pub fn metric(&self) -> Arc<MetricState> {
        self.metric.load().base.clone()
    }

    /// The current metric that avoids `area`, or `None` if no such area is loaded.
    /// Customized on first use for the current traffic state.
    pub fn metric_avoiding(&self, area: &str) -> Option<Arc<MetricState>> {
        let i = self.areas.iter().position(|a| a.name == area)?;
        let ms = self.metric.load_full();
        Some(
            ms.avoid[i]
                .get_or_init(|| {
                    let m = Arc::new(derive_avoid_metric(&self.graph, &ms.base, &self.areas[i].segments));
                    tracing::info!(area, "customized avoid metric in {:.0} ms", m.customize_ms);
                    m
                })
                .clone(),
        )
    }

    /// Input-graph arcs with their weights under metric `m` (for queries and unpacking).
    pub fn input_arcs<'a>(&'a self, m: &'a MetricState) -> InputArcs<'a> {
        InputArcs {
            first_out: &self.graph.first_out,
            head: &self.graph.head,
            turn: &self.graph.turn,
            elem_w: &m.elem_w,
            elem_entry: &m.elem_entry,
            arc_skip: &m.arc_skip,
            node_based: self.graph.meta.node_based,
        }
    }

    pub fn area_names(&self) -> Vec<String> {
        self.areas.iter().map(|a| a.name.clone()).collect()
    }

    pub fn overrides(&self) -> Overrides {
        self.overrides.lock().clone()
    }

    /// Applies `change` to the overrides, re-customizes, and publishes the new metric.
    /// Returns the new metric version and customization time.
    pub fn update(&self, change: impl FnOnce(&mut Overrides)) -> (u64, f64) {
        let _guard = self.rebuild.lock();
        let ov = {
            let mut o = self.overrides.lock();
            change(&mut o);
            o.clone()
        };
        let version = self.metric.load().base.version + 1;
        let t = Instant::now();
        let metrics = build_metrics(&self.graph, &self.areas, &ov, version);
        let ms = t.elapsed().as_secs_f64() * 1e3;
        self.metric.store(Arc::new(metrics));
        (version, ms)
    }

    /// Traffic lights a leg waits at (one per intersection), in travel order.
    pub fn signals_on(&self, m: &MetricState, pieces: &[Piece]) -> Vec<u32> {
        crate::signals::route_lights(&self.graph, &self.signal_index, &m.signal_waits, pieces)
    }

    pub fn snap(&self, c: Coord) -> Option<Snap> {
        self.graph.snap(c, MAX_SNAP_M)
    }

    fn with_buffers<R>(&self, f: impl FnOnce(&mut QueryBuffers) -> R) -> R {
        let mut b = self.buffers.lock().pop().unwrap_or_default();
        let r = f(&mut b);
        self.buffers.lock().push(b);
        r
    }

    /// Shortest path between two snapped positions under metric `m`.
    pub fn leg(&self, m: &MetricState, a: &Snap, b: &Snap) -> Option<LegPath> {
        if self.graph.meta.node_based {
            return self.leg_node_based(m, a, b);
        }
        let g = &self.graph;
        let dirs = |s: &Snap| {
            let seg = s.segment;
            let fraction = s.fraction;
            [true, false].into_iter().filter_map(move |fwd| {
                let node = g.seg_elem(seg, fwd);
                if node == NO_NODE || g.elem_flags[node as usize] & NODE_ROUTABLE == 0 {
                    return None;
                }
                let d = m.elem_w[node as usize];
                if d >= INF {
                    return None;
                }
                let pos = if fwd { fraction } else { 1.0 - fraction };
                Some((node, fwd, pos, d))
            })
        };
        let mut sources = Vec::with_capacity(2);
        let mut src_pos = Vec::with_capacity(2);
        for (node, _, pos, d) in dirs(a) {
            sources.push((node, ((1.0 - pos) * d as f64).round() as u32));
            src_pos.push(pos);
        }
        let mut offsets = Vec::with_capacity(2);
        let mut tgt_pos = Vec::with_capacity(2);
        for (node, _, pos, d) in dirs(b) {
            offsets.push((node, ((1.0 - pos) * d as f64).round() as u32));
            tgt_pos.push(pos);
        }
        let shift = offsets.iter().map(|o| o.1).max().unwrap_or(0);
        let targets: Vec<(u32, u32)> = offsets.iter().map(|&(n, o)| (n, shift - o)).collect();
        let mut no_trivial = Vec::new();
        for (i, s) in sources.iter().enumerate() {
            for (j, t) in targets.iter().enumerate() {
                if s.0 == t.0 && tgt_pos[j] < src_pos[i] {
                    no_trivial.push(s.0);
                }
            }
        }

        let arcs = self.input_arcs(m);
        let found = self.with_buffers(|buf| {
            query(&g.topo, &m.metric, &arcs, buf, &sources, &targets, &no_trivial)
        })?;
        if found.dist >= INF as u64 {
            return None;
        }

        let sp = src_pos[found.source];
        let tp = tgt_pos[found.target];
        let last = found.path.len() - 1;
        let mut pieces = Vec::with_capacity(found.path.len());
        for (k, &node) in found.path.iter().enumerate() {
            let (segment, forward) = g.elem_segment(node);
            let turn_ds = if k == 0 {
                0
            } else {
                let prev = found.path[k - 1];
                let r = g.first_out[prev as usize] as usize..g.first_out[prev as usize + 1] as usize;
                let arcs = self.input_arcs(m);
                r.filter(|&i| g.head[i] == node)
                    .map(|i| arcs.weight(i).saturating_sub(m.elem_w[node as usize]))
                    .min()
                    .unwrap_or(0)
            };
            pieces.push(Piece {
                elem: node,
                segment,
                forward,
                from: if k == 0 { sp } else { 0.0 },
                to: if k == last { tp } else { 1.0 },
                turn_ds,
            });
        }
        Some(LegPath {
            pieces,
            weight_ds: found.dist.saturating_sub(shift as u64),
        })
    }

    /// Leg on a node-based graph: the search runs between junctions. A start on segment s
    /// reaches the junction ahead of it (per allowed direction) after the rest of s; a target
    /// is reached from the junction behind it plus the part of its segment up to it. Starting
    /// and ending on the same segment is also tried directly.
    fn leg_node_based(&self, m: &MetricState, a: &Snap, b: &Snap) -> Option<LegPath> {
        let g = &self.graph;
        // (element, forward, position along direction, weight)
        let dirs = |s: &Snap| {
            let (seg, fraction) = (s.segment, s.fraction);
            [true, false].into_iter().filter_map(move |fwd| {
                let e = g.seg_elem(seg, fwd);
                if e == NO_NODE || g.elem_flags[e as usize] & NODE_ROUTABLE == 0 {
                    return None;
                }
                let w = m.elem_w[e as usize];
                (w < INF).then(|| (e, fwd, if fwd { fraction } else { 1.0 - fraction }, w))
            })
        };
        let src: Vec<_> = dirs(a).collect();
        let tgt: Vec<_> = dirs(b).collect();
        let sources: Vec<(u32, u32)> = src
            .iter()
            .map(|&(e, _, pos, w)| (g.head[e as usize], ((1.0 - pos) * w as f64).round() as u32))
            .collect();
        let targets: Vec<(u32, u32)> = tgt
            .iter()
            .map(|&(e, _, pos, w)| {
                // The arc's tail: the node whose out-arc range contains e.
                let tail = (g.first_out.partition_point(|&f| f <= e) - 1) as u32;
                (tail, (pos * w as f64).round() as u32)
            })
            .collect();

        // Direct: start and target on the same directed segment, target ahead of start.
        let mut best: Option<(u64, Vec<Piece>)> = None;
        if a.segment == b.segment {
            for &(e, fwd, ps, w) in &src {
                if let Some(&(_, _, pt, _)) = tgt.iter().find(|t| t.0 == e) {
                    if pt >= ps {
                        let cost = ((pt - ps) * w as f64).round() as u64;
                        if best.as_ref().is_none_or(|b| cost < b.0) {
                            let piece = Piece { elem: e, segment: a.segment, forward: fwd, from: ps, to: pt, turn_ds: 0 };
                            best = Some((cost, vec![piece]));
                        }
                    }
                }
            }
        }

        let arcs = self.input_arcs(m);
        let found = self.with_buffers(|buf| query(&g.topo, &m.metric, &arcs, buf, &sources, &targets, &[]));
        if let Some(f) = found.filter(|f| f.dist < INF as u64) {
            if best.as_ref().is_none_or(|b| f.dist < b.0) {
                let (se, sf, sp, _) = src[f.source];
                let (te, tf, tp, _) = tgt[f.target];
                let mut pieces = vec![Piece {
                    elem: se,
                    segment: dseg_seg(g.elem_dseg[se as usize]),
                    forward: sf,
                    from: sp,
                    to: 1.0,
                    turn_ds: 0,
                }];
                let mut prev_elem = se;
                for w in f.path.windows(2) {
                    let r = g.first_out[w[0] as usize] as usize..g.first_out[w[0] as usize + 1] as usize;
                    let e = r.filter(|&i| g.head[i] == w[1]).min_by_key(|&i| arcs.weight(i))? as u32;
                    let (segment, forward) = g.elem_segment(e);
                    pieces.push(Piece {
                        elem: e,
                        segment,
                        forward,
                        from: 0.0,
                        to: 1.0,
                        turn_ds: m.elem_entry[prev_elem as usize],
                    });
                    prev_elem = e;
                }
                pieces.push(Piece {
                    elem: te,
                    segment: dseg_seg(g.elem_dseg[te as usize]),
                    forward: tf,
                    from: 0.0,
                    to: tp,
                    turn_ds: m.elem_entry[prev_elem as usize],
                });
                best = Some((f.dist, pieces));
            }
        }
        let (weight_ds, pieces) = best?;
        Some(LegPath { pieces, weight_ds })
    }
}


/// Builds the base metric; area metrics start empty and are derived on demand.
pub fn build_metrics(g: &Graph, areas: &[AvoidArea], ov: &Overrides, version: u64) -> Metrics {
    Metrics {
        base: Arc::new(build_metric(g, ov, version)),
        avoid: areas.iter().map(|_| OnceLock::new()).collect(),
        overrides: Arc::new(ov.clone()),
    }
}

/// Computes element durations from the profile plus overrides (traffic, closures, light
/// waits), then customizes.
pub fn build_metric(g: &Graph, ov: &Overrides, version: u64) -> MetricState {
    let t = Instant::now();
    let idx = crate::signals::SignalIndex::new(g);
    let waits = crate::signals::waits(g, ov);
    let elem_dur: Vec<u32> = (0..g.elem_dseg.len() as u32)
        .into_par_iter()
        .map(|e| {
            let d = g.elem_dseg[e as usize];
            let seg_id = d >> 1;
            let fwd = d & 1 == 0;
            let seg = &g.segments[seg_id as usize];
            if ov.closed.contains(&seg_id) {
                return INF;
            }
            let k = (!fwd) as usize;
            let mut dur = seg.duration(fwd);
            if let Some(Some(kmh)) = ov.speed.get(&seg_id).map(|s| s[k]) {
                dur = if kmh <= 0.0 {
                    INF
                } else {
                    ((seg.length_m as f64 * 36.0 / kmh as f64).ceil() as u32).max(1)
                };
            }
            // Manual factor and live-feed factor combine (the feed is replaced each poll).
            let factor = ov.factor.get(&seg_id).and_then(|s| s[k]).unwrap_or(1.0)
                * ov.feed_factor.get(&seg_id).and_then(|s| s[k]).unwrap_or(1.0);
            if factor != 1.0 {
                dur = if factor <= 0.0 { INF } else { ((dur as f64 * factor as f64).ceil() as u32).max(1) };
            }
            if dur >= INF {
                return INF;
            }
            (dur + crate::signals::interior_wait(g, &idx, &waits, e)).min(INF)
        })
        .collect();

    // Junction lights. Edge-based: the wait is put on the node entered (its start junction)
    // and skipped on arcs whose tail is already inside that intersection. Node-based: the
    // arc itself passes its end junction.
    let node_based = g.meta.node_based;
    let junction_wait = |j: u32| idx.by_junction.get(&j).map_or(0, |&s| waits[s as usize]);
    let elem_entry: Vec<u32> = (0..g.elem_dseg.len() as u32)
        .into_par_iter()
        .map(|e| {
            if node_based {
                crate::signals::end_junction_signal(g, &idx, &waits, e).map_or(0, |s| waits[s as usize])
            } else {
                junction_wait(g.elem_start_junction(e))
            }
        })
        .collect();
    let arc_skip: Vec<u64> = if node_based {
        Vec::new()
    } else {
        let tail_pays: Vec<bool> = (0..g.elem_dseg.len() as u32)
            .into_par_iter()
            .map(|e| crate::signals::end_junction_signal(g, &idx, &waits, e).is_some())
            .collect();
        let mut bits = vec![0u64; g.head.len().div_ceil(64)];
        for (a, pays) in tail_pays.iter().enumerate() {
            if !pays {
                for i in g.first_out[a] as usize..g.first_out[a + 1] as usize {
                    bits[i / 64] |= 1 << (i % 64);
                }
            }
        }
        bits
    };
    let elem_dur = Arc::new(elem_dur);
    customize_with(
        g,
        elem_dur.clone(),
        elem_dur,
        Arc::new(elem_entry),
        Arc::new(arc_skip),
        Arc::new(waits),
        version,
        t,
    )
}

/// Derives the metric that avoids an area from a base metric: same real durations, but
/// travel inside the area weighs `AVOID_FACTOR` times more.
pub fn derive_avoid_metric(g: &Graph, base: &MetricState, inside: &[u64]) -> MetricState {
    let t = Instant::now();
    let elem_w: Vec<u32> = g
        .elem_dseg
        .par_iter()
        .zip(base.elem_dur.par_iter())
        .map(|(&d, &dur)| {
            let s = (d >> 1) as usize;
            if dur < INF && inside[s / 64] & (1 << (s % 64)) != 0 {
                ((dur as f64 * crate::area::AVOID_FACTOR) as u32).min(INF - 1)
            } else {
                dur
            }
        })
        .collect();
    customize_with(
        g,
        base.elem_dur.clone(),
        Arc::new(elem_w),
        base.elem_entry.clone(),
        base.arc_skip.clone(),
        base.signal_waits.clone(),
        base.version,
        t,
    )
}

#[allow(clippy::too_many_arguments)]
fn customize_with(
    g: &Graph,
    elem_dur: Arc<Vec<u32>>,
    elem_w: Arc<Vec<u32>>,
    elem_entry: Arc<Vec<u32>>,
    arc_skip: Arc<Vec<u64>>,
    signal_waits: Arc<Vec<u32>>,
    version: u64,
    t: Instant,
) -> MetricState {
    let metric = {
        let arcs = InputArcs {
            first_out: &g.first_out,
            head: &g.head,
            turn: &g.turn,
            elem_w: &elem_w,
            elem_entry: &elem_entry,
            arc_skip: &arc_skip,
            node_based: g.meta.node_based,
        };
        customize(&g.topo, &g.arc_to_cch, |i| arcs.weight(i))
    };
    MetricState {
        metric,
        elem_dur,
        elem_w,
        elem_entry,
        arc_skip,
        signal_waits,
        version,
        customize_ms: t.elapsed().as_secs_f64() * 1e3,
    }
}
