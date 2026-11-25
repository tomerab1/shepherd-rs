//! Reference shortest paths (plain Dijkstra on the edge-based graph) for verifying the CCH.

use crate::cch::query::{query, QueryBuffers};
use crate::cch::INF;
use crate::engine::{Engine, MetricState};
use std::cmp::Reverse;
use std::collections::BinaryHeap;

/// One-to-all Dijkstra distances from EBG node `s` (node weights are on arcs).
pub fn dijkstra(engine: &Engine, m: &MetricState, s: u32) -> Vec<u32> {
    let g = &engine.graph;
    let n = g.first_out.len() - 1;
    let mut dist = vec![INF; n];
    let mut heap = BinaryHeap::new();
    dist[s as usize] = 0;
    heap.push(Reverse((0u32, s)));
    while let Some(Reverse((d, v))) = heap.pop() {
        if d > dist[v as usize] {
            continue;
        }
        let arcs = engine.input_arcs(m);
        for i in g.first_out[v as usize] as usize..g.first_out[v as usize + 1] as usize {
            let w = arcs.weight(i);
            if w >= INF {
                continue;
            }
            let nd = d + w;
            let h = g.head[i] as usize;
            if nd < dist[h] {
                dist[h] = nd;
                heap.push(Reverse((nd, h as u32)));
            }
        }
    }
    dist
}

/// CCH node-to-node distance (no snapping offsets).
pub fn cch_distance(engine: &Engine, m: &MetricState, buf: &mut QueryBuffers, s: u32, t: u32) -> u32 {
    let g = &engine.graph;
    let arcs = engine.input_arcs(m);
    query(&g.topo, &m.metric, &arcs, buf, &[(s, 0)], &[(t, 0)], &[])
        .map(|f| f.dist.min(INF as u64) as u32)
        .unwrap_or(INF)
}

/// Checks `pairs` random source/target pairs; returns the number of mismatches.
/// Also checks that each unpacked path is a real path whose arc weights sum to the distance.
pub fn verify(engine: &Engine, sources: usize, targets_per_source: usize, seed: u64) -> usize {
    let m = engine.metric();
    let g = &engine.graph;
    let n = (g.first_out.len() - 1) as u64;
    let mut rng = seed | 1;
    let mut next = || {
        rng ^= rng << 13;
        rng ^= rng >> 7;
        rng ^= rng << 17;
        (rng % n) as u32
    };
    let mut buf = QueryBuffers::default();
    let mut bad = 0;
    let arcs = engine.input_arcs(&m);
    for _ in 0..sources {
        let s = next();
        let reference = dijkstra(engine, &m, s);
        for _ in 0..targets_per_source {
            let t = next();
            let found = query(&g.topo, &m.metric, &arcs, &mut buf, &[(s, 0)], &[(t, 0)], &[]);
            let d = found.as_ref().map(|f| f.dist.min(INF as u64) as u32).unwrap_or(INF);
            if d != reference[t as usize] {
                bad += 1;
                tracing::error!(s, t, cch = d, dijkstra = reference[t as usize], "distance mismatch");
                continue;
            }
            if let Some(f) = found {
                let mut sum = 0u64;
                for w in f.path.windows(2) {
                    let r = g.first_out[w[0] as usize] as usize..g.first_out[w[0] as usize + 1] as usize;
                    match r.filter(|&i| g.head[i] == w[1]).map(|i| arcs.weight(i)).min() {
                        Some(a) => sum += a as u64,
                        None => {
                            bad += 1;
                            tracing::error!(s, t, "unpacked path uses a non-existent arc");
                            break;
                        }
                    }
                }
                if f.path[0] != s || *f.path.last().unwrap() != t || sum != d as u64 {
                    bad += 1;
                    tracing::error!(s, t, sum, d, "unpacked path does not match distance");
                }
            }
        }
    }
    bad
}
