//! Elimination-tree query.
//!
//! In a CCH every upward neighbour of a node is one of its ancestors in the elimination tree.
//! So the forward search space of `s` is exactly the path from `s` to the root, and we can
//! relax nodes in increasing rank without a priority queue. Search state is stored per
//! ancestor (a few hundred to a few thousand entries), not per graph node, so a query needs
//! only kilobytes of scratch memory and no per-thread `O(n)` arrays.

use super::customize::Metric;
use super::{Topology, INF, NONE};

#[derive(Default)]
struct Side {
    nodes: Vec<u32>,
    dist: Vec<u32>,
    pred: Vec<u32>,
}

impl Side {
    fn init(&mut self, topo: &Topology, starts: &[(u32, u32)]) {
        self.nodes.clear();
        for &(s, _) in starts {
            let mut x = s;
            while x != NONE {
                self.nodes.push(x);
                x = topo.parent[x as usize];
            }
        }
        self.nodes.sort_unstable();
        self.nodes.dedup();
        self.dist.clear();
        self.dist.resize(self.nodes.len(), INF);
        self.pred.clear();
        self.pred.resize(self.nodes.len(), NONE);
        for &(s, d) in starts {
            let i = self.nodes.binary_search(&s).unwrap();
            self.dist[i] = self.dist[i].min(d);
        }
    }

    /// Relaxes the upward edges of the node at index `i`.
    #[inline]
    fn relax(&mut self, topo: &Topology, weights: &[u32], i: usize, bound: u64) {
        let d = self.dist[i];
        if d >= INF || d as u64 >= bound {
            return;
        }
        let x = self.nodes[i];
        let mut cursor = i + 1;
        for e in topo.up(x) {
            let wt = weights[e];
            if wt >= INF {
                continue;
            }
            let w = topo.up_head[e];
            let j = gallop(&self.nodes, cursor, w);
            debug_assert_eq!(self.nodes[j], w, "upward neighbour is not an ancestor");
            cursor = j + 1;
            let nd = (d + wt).min(INF);
            if nd < self.dist[j] {
                self.dist[j] = nd;
                self.pred[j] = i as u32;
            }
        }
    }
}

/// Index of `x` in sorted `v`, searching forward from `start`.
#[inline]
fn gallop(v: &[u32], start: usize, x: u32) -> usize {
    let mut lo = start;
    let mut step = 1;
    let mut hi = start;
    while hi < v.len() && v[hi] < x {
        lo = hi;
        hi += step;
        step <<= 1;
    }
    let hi = hi.min(v.len() - 1) + 1;
    lo + v[lo..hi].partition_point(|&y| y < x)
}

/// Reusable scratch space for queries (one per worker).
#[derive(Default)]
pub struct QueryBuffers {
    fwd: Side,
    bwd: Side,
    stack: Vec<(u32, u32)>,
    cch_path: Vec<u32>,
}

pub struct Found {
    /// Shortest distance including the start/target offsets.
    pub dist: u64,
    /// EBG nodes from source to target.
    pub path: Vec<u32>,
    pub source: usize,
    pub target: usize,
}

/// Graph data the query needs besides the CCH topology.
pub struct InputArcs<'a> {
    pub first_out: &'a [u32],
    pub head: &'a [u32],
    pub turn: &'a [u16],
    /// Weight of traversing each node (directed segment).
    pub elem_w: &'a [u32],
    /// Extra weight when entering a node from its start junction (traffic light there).
    pub elem_entry: &'a [u32],
    /// Edge-based graphs: arcs that skip the light wait of the node they enter.
    pub arc_skip: &'a [u64],
    /// Node-based graph: elements are the arcs themselves (`elem_w[i] + elem_entry[i]`).
    pub node_based: bool,
}

impl InputArcs<'_> {
    /// Arc weight: `elem_w[head] + elem_entry[head] + turn`.
    #[inline]
    pub fn weight(&self, i: usize) -> u32 {
        if self.node_based {
            return (self.elem_w[i].min(INF) + self.elem_entry[i]).min(INF);
        }
        let h = self.head[i] as usize;
        let skip = self.arc_skip.get(i / 64).is_some_and(|w| w & (1 << (i % 64)) != 0);
        let entry = if skip { 0 } else { self.elem_entry[h] };
        (self.elem_w[h].min(INF) + entry + self.turn[i] as u32).min(INF)
    }
}

/// Shortest path from any of `sources` to any of `targets`, each given as
/// `(node, initial distance)`. Nodes in `no_trivial` are both a source and a target where the
/// zero-length path (start == target node) is not allowed (target lies behind the start on a
/// one-way segment), so a real loop must be found instead.
pub fn query(
    topo: &Topology,
    metric: &Metric,
    arcs: &InputArcs,
    buf: &mut QueryBuffers,
    sources: &[(u32, u32)],
    targets: &[(u32, u32)],
    no_trivial: &[u32],
) -> Option<Found> {
    if sources.is_empty() || targets.is_empty() {
        return None;
    }
    buf.fwd.init(topo, sources);
    buf.bwd.init(topo, targets);
    let (fwd, bwd) = (&mut buf.fwd, &mut buf.bwd);

    let mut best = u64::MAX;
    let mut meet = None;
    let (mut i, mut j) = (0usize, 0usize);
    while i < fwd.nodes.len() || j < bwd.nodes.len() {
        let xf = fwd.nodes.get(i).copied().unwrap_or(u32::MAX);
        let xb = bwd.nodes.get(j).copied().unwrap_or(u32::MAX);
        if xf == xb {
            let (df, db) = (fwd.dist[i], bwd.dist[j]);
            if df < INF && db < INF {
                let trivial = fwd.pred[i] == NONE && bwd.pred[j] == NONE && no_trivial.contains(&xf);
                let total = df as u64 + db as u64;
                if !trivial && total < best {
                    best = total;
                    meet = Some((i, j));
                }
            }
            fwd.relax(topo, &metric.up, i, best);
            bwd.relax(topo, &metric.down, j, best);
            i += 1;
            j += 1;
        } else if xf < xb {
            fwd.relax(topo, &metric.up, i, best);
            i += 1;
        } else {
            bwd.relax(topo, &metric.down, j, best);
            j += 1;
        }
    }
    let (mi, mj) = meet?;

    // Path in the CCH: source .. meeting node .. target.
    let path = &mut buf.cch_path;
    path.clear();
    let mut k = mi as u32;
    while k != NONE {
        path.push(fwd.nodes[k as usize]);
        k = fwd.pred[k as usize];
    }
    path.reverse();
    let mut k = bwd.pred[mj];
    while k != NONE {
        path.push(bwd.nodes[k as usize]);
        k = bwd.pred[k as usize];
    }

    let first = path[0];
    let last = *path.last().unwrap();
    let source = pick(sources, first);
    let target = pick(targets, last);

    let mut out = Vec::with_capacity(path.len() * 4);
    out.push(first);
    for w in path.windows(2) {
        unpack(topo, metric, arcs, w[0], w[1], &mut buf.stack, &mut out);
    }
    Some(Found {
        dist: best,
        path: out,
        source,
        target,
    })
}

/// Index of the start entry for `node` (the one with the smallest initial distance).
fn pick(list: &[(u32, u32)], node: u32) -> usize {
    list.iter()
        .enumerate()
        .filter(|(_, &(n, _))| n == node)
        .min_by_key(|(_, &(_, d))| d)
        .map_or(0, |(i, _)| i)
}

/// Appends the input-graph path for CCH edge a -> b (excluding `a`) to `out`.
fn unpack(
    topo: &Topology,
    metric: &Metric,
    arcs: &InputArcs,
    a: u32,
    b: u32,
    stack: &mut Vec<(u32, u32)>,
    out: &mut Vec<u32>,
) {
    stack.clear();
    stack.push((a, b));
    while let Some((a, b)) = stack.pop() {
        let w = if a < b {
            metric.up[topo.find_edge(a, b).unwrap() as usize]
        } else {
            metric.down[topo.find_edge(b, a).unwrap() as usize]
        };
        // An input arc with exactly this weight?
        let range = arcs.first_out[a as usize] as usize..arcs.first_out[a as usize + 1] as usize;
        if range.clone().any(|i| arcs.head[i] == b && arcs.weight(i) == w) {
            out.push(b);
            continue;
        }
        // Otherwise a lower triangle a -> u -> b with u below both: intersect the (sorted)
        // down lists of a and b, which yields both edge ids without any search.
        let lo = a.min(b);
        let (ra, rb) = (topo.down(a), topo.down(b));
        let (mut i, mut j) = (ra.start, rb.start);
        let mut found = false;
        while i < ra.end && j < rb.end {
            let (ua, ub) = (topo.down_tail[i], topo.down_tail[j]);
            if ua >= lo || ub >= lo {
                break;
            }
            if ua < ub {
                i += 1;
            } else if ub < ua {
                j += 1;
            } else {
                // a -> u uses edge (u, a) downward; u -> b uses edge (u, b) upward.
                let w1 = metric.down[topo.down_edge[i] as usize];
                let w2 = metric.up[topo.down_edge[j] as usize];
                if w1 < INF && w2 < INF && w1 + w2 == w {
                    stack.push((ua, b));
                    stack.push((a, ua));
                    found = true;
                    break;
                }
                i += 1;
                j += 1;
            }
        }
        assert!(found, "CCH unpacking failed for edge {a} -> {b}");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gallop_finds_values() {
        let v: Vec<u32> = (0..1000).map(|x| x * 3).collect();
        for s in [0, 5, 100] {
            for x in (s as u32 * 3..2997).step_by(7).filter(|x| x % 3 == 0) {
                assert_eq!(v[gallop(&v, s, x)], x);
            }
        }
    }
}
