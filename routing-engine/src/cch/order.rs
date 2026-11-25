//! Metric-independent node order by nested dissection with inertial-flow separators.
//!
//! Recursively: project the nodes onto a line, treat the first/last 25% as source/sink, find a
//! minimum *vertex* cut between them with unit-capacity max-flow, order both halves first and
//! the separator last. Small separators near the top of the order are what make CCH fast.
//! The two halves are independent, so they are processed in parallel.

use std::collections::VecDeque;

/// An undirected graph with planar coordinates, as a local CSR.
pub struct SubGraph {
    /// Global id of each local node.
    pub ids: Vec<u32>,
    pub xy: Vec<[f32; 2]>,
    pub first: Vec<u32>,
    pub adj: Vec<u32>,
}

impl SubGraph {
    fn n(&self) -> usize {
        self.ids.len()
    }
    #[inline]
    fn nbrs(&self, v: usize) -> &[u32] {
        &self.adj[self.first[v] as usize..self.first[v + 1] as usize]
    }

    /// Builds an undirected, deduplicated, self-loop-free graph from directed edges.
    pub fn from_edges(n: usize, xy: Vec<[f32; 2]>, edges: impl Iterator<Item = (u32, u32)>) -> Self {
        let mut pairs: Vec<(u32, u32)> = Vec::new();
        for (a, b) in edges {
            if a != b {
                pairs.push((a, b));
                pairs.push((b, a));
            }
        }
        pairs.sort_unstable();
        pairs.dedup();
        let mut first = vec![0u32; n + 1];
        for &(a, _) in &pairs {
            first[a as usize + 1] += 1;
        }
        for i in 0..n {
            first[i + 1] += first[i];
        }
        SubGraph {
            ids: (0..n as u32).collect(),
            xy,
            first,
            adj: pairs.into_iter().map(|p| p.1).collect(),
        }
    }

    /// Induced subgraph on the local nodes in `keep` (in that order).
    fn induced(&self, keep: &[u32], local: &mut [u32]) -> SubGraph {
        const NONE: u32 = u32::MAX;
        for (i, &v) in keep.iter().enumerate() {
            local[v as usize] = i as u32;
        }
        let mut first = Vec::with_capacity(keep.len() + 1);
        let mut adj = Vec::new();
        first.push(0);
        for &v in keep {
            for &w in self.nbrs(v as usize) {
                let lw = local[w as usize];
                if lw != NONE {
                    adj.push(lw);
                }
            }
            first.push(adj.len() as u32);
        }
        let g = SubGraph {
            ids: keep.iter().map(|&v| self.ids[v as usize]).collect(),
            xy: keep.iter().map(|&v| self.xy[v as usize]).collect(),
            first,
            adj,
        };
        for &v in keep {
            local[v as usize] = NONE;
        }
        g
    }

    fn components(&self) -> Vec<Vec<u32>> {
        let n = self.n();
        let mut seen = vec![false; n];
        let mut comps = Vec::new();
        let mut queue = VecDeque::new();
        for s in 0..n {
            if seen[s] {
                continue;
            }
            let mut comp = vec![s as u32];
            seen[s] = true;
            queue.push_back(s as u32);
            while let Some(v) = queue.pop_front() {
                for &w in self.nbrs(v as usize) {
                    if !seen[w as usize] {
                        seen[w as usize] = true;
                        comp.push(w);
                        queue.push_back(w);
                    }
                }
            }
            comps.push(comp);
        }
        comps
    }
}

const LEAF_SIZE: usize = 8;
const PARALLEL_CUTOFF: usize = 2_000;
const TERMINAL_FRACTION: f64 = 0.25;
const DIRECTIONS: [[f32; 2]; 4] = [[1.0, 0.0], [0.0, 1.0], [0.707, 0.707], [0.707, -0.707]];

/// Returns global node ids from lowest to highest rank.
pub fn nested_dissection(g: SubGraph) -> Vec<u32> {
    let mut out = Vec::with_capacity(g.n());
    dissect(g, &mut out);
    out
}

fn dissect(g: SubGraph, out: &mut Vec<u32>) {
    let n = g.n();
    if n <= LEAF_SIZE {
        // Lowest degree first keeps fill-in down inside tiny cells.
        let mut v: Vec<usize> = (0..n).collect();
        v.sort_by_key(|&i| g.first[i + 1] - g.first[i]);
        out.extend(v.into_iter().map(|i| g.ids[i]));
        return;
    }
    let mut local = vec![u32::MAX; n];
    let comps = g.components();
    if comps.len() > 1 {
        let subs: Vec<SubGraph> = comps.iter().map(|c| g.induced(c, &mut local)).collect();
        drop(g);
        let parts: Vec<Vec<u32>> = if n >= PARALLEL_CUTOFF {
            use rayon::prelude::*;
            subs.into_par_iter().map(nested_dissection).collect()
        } else {
            subs.into_iter().map(nested_dissection).collect()
        };
        for p in parts {
            out.extend(p);
        }
        return;
    }

    let cut = best_cut(&g);
    let (a, b) = {
        let side_a: Vec<u32> = (0..n as u32).filter(|&v| cut.side[v as usize] == Side::A).collect();
        let side_b: Vec<u32> = (0..n as u32).filter(|&v| cut.side[v as usize] == Side::B).collect();
        (g.induced(&side_a, &mut local), g.induced(&side_b, &mut local))
    };
    let separator: Vec<u32> = (0..n)
        .filter(|&v| cut.side[v] == Side::Sep)
        .map(|v| g.ids[v])
        .collect();
    drop(g);

    let (oa, ob) = if n >= PARALLEL_CUTOFF {
        rayon::join(|| nested_dissection(a), || nested_dissection(b))
    } else {
        (nested_dissection(a), nested_dissection(b))
    };
    out.extend(oa);
    out.extend(ob);
    out.extend(separator);
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Side {
    A,
    B,
    Sep,
}

struct Cut {
    side: Vec<Side>,
    size: usize,
    balance: usize,
}

fn best_cut(g: &SubGraph) -> Cut {
    let cuts: Vec<Cut> = if g.n() >= PARALLEL_CUTOFF * 4 {
        use rayon::prelude::*;
        DIRECTIONS.par_iter().map(|d| inertial_cut(g, *d)).collect()
    } else {
        DIRECTIONS.iter().map(|d| inertial_cut(g, *d)).collect()
    };
    cuts.into_iter()
        .min_by(|x, y| {
            // Prefer small separators; among similar ones prefer balanced halves.
            let sx = x.size as f64 / (x.balance.max(1) as f64).sqrt();
            let sy = y.size as f64 / (y.balance.max(1) as f64).sqrt();
            sx.partial_cmp(&sy).unwrap()
        })
        .unwrap()
}

/// Minimum vertex cut between the first and last 25% of nodes along direction `dir`.
/// Unit-capacity max-flow on the node-split graph, augmenting along BFS paths.
fn inertial_cut(g: &SubGraph, dir: [f32; 2]) -> Cut {
    let n = g.n();
    let mut by_proj: Vec<u32> = (0..n as u32).collect();
    by_proj.sort_unstable_by(|&a, &b| {
        let pa = g.xy[a as usize][0] * dir[0] + g.xy[a as usize][1] * dir[1];
        let pb = g.xy[b as usize][0] * dir[0] + g.xy[b as usize][1] * dir[1];
        pa.partial_cmp(&pb).unwrap()
    });
    let k = ((n as f64 * TERMINAL_FRACTION) as usize).max(1);
    // 0 = normal, 1 = source, 2 = sink
    let mut term = vec![0u8; n];
    for &v in &by_proj[..k] {
        term[v as usize] = 1;
    }
    for &v in &by_proj[n - k..] {
        term[v as usize] = 2;
    }

    // Reverse arc index for each arc.
    let mut rev = vec![0u32; g.adj.len()];
    for v in 0..n {
        for i in g.first[v] as usize..g.first[v + 1] as usize {
            let w = g.adj[i] as usize;
            let nb = g.nbrs(w);
            let j = nb.binary_search(&(v as u32)).unwrap_or_else(|_| {
                nb.iter().position(|&x| x == v as u32).unwrap()
            });
            rev[i] = g.first[w] + j as u32;
        }
    }

    let mut node_flow = vec![false; n]; // internal arc v_in -> v_out saturated
    let mut arc_flow = vec![0i32; g.adj.len()]; // flow on u_out -> v_in

    // BFS state: 2*v = v_in, 2*v+1 = v_out
    const UNSEEN: u32 = u32::MAX;
    let mut parent = vec![UNSEEN; 2 * n];
    // How we reached a state: arc index (for inter-node moves) or u32::MAX for internal moves.
    let mut via = vec![u32::MAX; 2 * n];
    let mut queue: VecDeque<u32> = VecDeque::new();
    let mut flow = 0usize;

    loop {
        parent.fill(UNSEEN);
        queue.clear();
        for v in 0..n {
            if term[v] == 1 {
                let s = 2 * v as u32;
                parent[s as usize] = s;
                queue.push_back(s);
            }
        }
        let mut found = None;
        'bfs: while let Some(s) = queue.pop_front() {
            let v = (s / 2) as usize;
            let is_out = s & 1 == 1;
            let mut visit = |t: u32, arc: u32, queue: &mut VecDeque<u32>| -> bool {
                if parent[t as usize] == UNSEEN {
                    parent[t as usize] = s;
                    via[t as usize] = arc;
                    queue.push_back(t);
                    return true;
                }
                false
            };
            if is_out {
                // v_out -> w_in along any arc (infinite capacity).
                for i in g.first[v] as usize..g.first[v + 1] as usize {
                    let w = g.adj[i];
                    visit(2 * w, i as u32, &mut queue);
                }
                // Undo internal flow: v_out -> v_in.
                if node_flow[v] {
                    visit(2 * v as u32, u32::MAX, &mut queue);
                }
            } else {
                // v_in -> v_out if capacity left. Every node, terminals included, has capacity
                // 1 so an edge between a source and a sink cannot carry unbounded flow; the
                // super-sink is attached at the sinks' out side.
                if !node_flow[v] && visit(2 * v as u32 + 1, u32::MAX, &mut queue) && term[v] == 2 {
                    found = Some(2 * v as u32 + 1);
                    break 'bfs;
                }
                // v_in -> u_out: cancel flow on u_out -> v_in.
                for i in g.first[v] as usize..g.first[v + 1] as usize {
                    let j = rev[i] as usize; // arc u -> v
                    if arc_flow[j] > 0 {
                        let u = g.adj[i];
                        visit(2 * u + 1, j as u32 | (1 << 31), &mut queue);
                    }
                }
            }
        }

        let Some(mut t) = found else { break };
        flow += 1;
        // Augment along the path.
        while parent[t as usize] != t {
            let p = parent[t as usize];
            let arc = via[t as usize];
            if arc == u32::MAX {
                // in->out saturates, out->in unsaturates.
                node_flow[(t / 2) as usize] = t & 1 == 1;
            } else if arc & (1 << 31) != 0 {
                arc_flow[(arc & !(1 << 31)) as usize] -= 1;
            } else {
                let i = arc as usize;
                if arc_flow[rev[i] as usize] > 0 {
                    arc_flow[rev[i] as usize] -= 1;
                } else {
                    arc_flow[i] += 1;
                }
            }
            t = p;
        }
    }

    // Final BFS reachability defines the cut: in reached but out not reached => separator.
    let mut side = vec![Side::B; n];
    let mut a_count = 0;
    let mut sep = 0;
    for v in 0..n {
        let r_in = parent[2 * v] != UNSEEN;
        let r_out = parent[2 * v + 1] != UNSEEN;
        side[v] = match (r_in, r_out) {
            (_, true) => {
                a_count += 1;
                Side::A
            }
            (true, false) => {
                sep += 1;
                Side::Sep
            }
            _ => Side::B,
        };
    }
    debug_assert_eq!(sep, flow);
    let b_count = n - a_count - sep;
    Cut {
        side,
        size: sep,
        balance: a_count.min(b_count),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn grid_order_is_a_permutation_with_small_top_separator() {
        let w = 30;
        let h = 30;
        let id = |x: usize, y: usize| (y * w + x) as u32;
        let mut edges = Vec::new();
        let mut xy = Vec::new();
        for y in 0..h {
            for x in 0..w {
                xy.push([x as f32, y as f32]);
                if x + 1 < w {
                    edges.push((id(x, y), id(x + 1, y)));
                }
                if y + 1 < h {
                    edges.push((id(x, y), id(x, y + 1)));
                }
            }
        }
        let g = SubGraph::from_edges(w * h, xy, edges.iter().copied());
        let order = nested_dissection(g);
        let mut sorted = order.clone();
        sorted.sort();
        assert_eq!(sorted, (0..(w * h) as u32).collect::<Vec<_>>());
        // Nested dissection on a 30x30 grid must give a shallow elimination tree and little
        // fill-in (a bad order gives depth ~900 and far more edges).
        let mut perm = vec![0u32; w * h];
        for (r, &v) in order.iter().enumerate() {
            perm[v as usize] = r as u32;
        }
        let topo = crate::cch::Topology::build(w * h, edges.iter().map(|&(a, b)| (perm[a as usize], perm[b as usize])));
        assert!(topo.max_depth() < 150, "depth {}", topo.max_depth());
        assert!(topo.num_edges() < 12 * w * h, "edges {}", topo.num_edges());
    }
}
