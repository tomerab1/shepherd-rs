//! Customizable Contraction Hierarchies.
//!
//! Three phases, which is what makes live traffic and road closures cheap:
//! 1. **Order** (slow, metric-independent): nested dissection, see [`order`].
//! 2. **Topology** (fast, metric-independent): the chordal supergraph of the graph under that
//!    order — every "shortcut" that *could* ever be needed — plus its elimination tree.
//! 3. **Customization** (sub-second, parallel): given travel times, compute the weight of every
//!    edge of that supergraph. Changing traffic only re-runs this phase. See [`customize`].
//!
//! Queries walk up the elimination tree from source and target; no priority queue is needed.
//! Node ids are ranks: node 0 is contracted first, node n-1 last.

pub mod customize;
pub mod order;
pub mod query;

use crate::store::Arr;

pub const NONE: u32 = u32::MAX;
/// "Infinite" weight. Two of them still fit in a u32 without overflow.
pub const INF: u32 = u32::MAX / 2;

/// The metric-independent part of a CCH.
pub struct Topology {
    /// Upward edges: for node v, edges (v, w) with w > v, sorted by w. Edge id = position.
    pub first_up: Arr<u32>,
    pub up_head: Arr<u32>,
    /// Downward view of the same edges: for node w, the (u, edge id) with u < w, sorted by u.
    pub first_down: Arr<u32>,
    pub down_tail: Arr<u32>,
    pub down_edge: Arr<u32>,
    /// Elimination tree parent (lowest upward neighbour) or `NONE` for roots.
    pub parent: Arr<u32>,
    /// Nodes grouped by level (all lower neighbours of a node are on lower levels);
    /// nodes of one level are customized in parallel.
    pub level_first: Arr<u32>,
    pub level_nodes: Arr<u32>,
}

impl Topology {
    #[inline]
    pub fn num_nodes(&self) -> usize {
        self.parent.len()
    }
    #[inline]
    pub fn num_edges(&self) -> usize {
        self.up_head.len()
    }
    #[inline]
    pub fn up(&self, v: u32) -> std::ops::Range<usize> {
        self.first_up[v as usize] as usize..self.first_up[v as usize + 1] as usize
    }
    #[inline]
    pub fn down(&self, v: u32) -> std::ops::Range<usize> {
        self.first_down[v as usize] as usize..self.first_down[v as usize + 1] as usize
    }
    /// Edge id of (lo, hi) with lo < hi, if it exists.
    #[inline]
    pub fn find_edge(&self, lo: u32, hi: u32) -> Option<u32> {
        let r = self.up(lo);
        self.up_head[r.clone()]
            .binary_search(&hi)
            .ok()
            .map(|i| (r.start + i) as u32)
    }

    /// Builds the chordal supergraph of the undirected graph given by `edges` on `n` ranked nodes.
    pub fn build(n: usize, edges: impl Iterator<Item = (u32, u32)>) -> Topology {
        let mut up: Vec<Vec<u32>> = vec![Vec::new(); n];
        for (a, b) in edges {
            if a != b {
                let (lo, hi) = if a < b { (a, b) } else { (b, a) };
                up[lo as usize].push(hi);
            }
        }
        for l in up.iter_mut() {
            l.sort_unstable();
            l.dedup();
        }
        // Contract in rank order: v's upper neighbours form a clique, which is represented by
        // merging them into the lowest one (the elimination tree parent).
        let mut parent = vec![NONE; n];
        let mut merged: Vec<u32> = Vec::new();
        for v in 0..n {
            if up[v].is_empty() {
                continue;
            }
            let p = up[v][0] as usize;
            parent[v] = p as u32;
            if up[v].len() > 1 {
                let (lo, hi) = up.split_at_mut(p);
                let src = &lo[v][1..];
                let dst = &hi[0];
                merged.clear();
                merged.reserve(src.len() + dst.len());
                let (mut i, mut j) = (0, 0);
                while i < src.len() || j < dst.len() {
                    let x = if j == dst.len() || (i < src.len() && src[i] < dst[j]) {
                        i += 1;
                        src[i - 1]
                    } else if i == src.len() || dst[j] < src[i] {
                        j += 1;
                        dst[j - 1]
                    } else {
                        i += 1;
                        j += 1;
                        src[i - 1]
                    };
                    merged.push(x);
                }
                std::mem::swap(&mut up[p], &mut merged);
            }
        }

        let m: usize = up.iter().map(Vec::len).sum();
        let mut first_up = Vec::with_capacity(n + 1);
        let mut up_head = Vec::with_capacity(m);
        first_up.push(0u32);
        for l in &up {
            up_head.extend_from_slice(l);
            first_up.push(up_head.len() as u32);
        }
        drop(up);

        // Downward CSR (counting sort by head).
        let mut first_down = vec![0u32; n + 1];
        for &w in &up_head {
            first_down[w as usize + 1] += 1;
        }
        for i in 0..n {
            first_down[i + 1] += first_down[i];
        }
        let mut fill = first_down.clone();
        let mut down_tail = vec![0u32; m];
        let mut down_edge = vec![0u32; m];
        for v in 0..n {
            for e in first_up[v] as usize..first_up[v + 1] as usize {
                let w = up_head[e] as usize;
                let k = fill[w] as usize;
                down_tail[k] = v as u32;
                down_edge[k] = e as u32;
                fill[w] += 1;
            }
        }

        // Levels for parallel customization.
        let mut level = vec![0u32; n];
        for v in 0..n {
            for e in first_up[v] as usize..first_up[v + 1] as usize {
                let w = up_head[e] as usize;
                level[w] = level[w].max(level[v] + 1);
            }
        }
        let nlevels = level.iter().copied().max().map_or(0, |l| l as usize + 1);
        let mut level_first = vec![0u32; nlevels + 1];
        for &l in &level {
            level_first[l as usize + 1] += 1;
        }
        for i in 0..nlevels {
            level_first[i + 1] += level_first[i];
        }
        let mut fill = level_first.clone();
        let mut level_nodes = vec![0u32; n];
        for (v, &l) in level.iter().enumerate() {
            level_nodes[fill[l as usize] as usize] = v as u32;
            fill[l as usize] += 1;
        }

        Topology {
            first_up: first_up.into(),
            up_head: up_head.into(),
            first_down: first_down.into(),
            down_tail: down_tail.into(),
            down_edge: down_edge.into(),
            parent: parent.into(),
            level_first: level_first.into(),
            level_nodes: level_nodes.into(),
        }
    }

    /// Maps directed input arcs (a -> b) to `edge << 1 | is_down`, or `NONE` for self-loops.
    pub fn map_arcs(&self, arcs: impl Iterator<Item = (u32, u32)>) -> Vec<u32> {
        arcs.map(|(a, b)| {
            if a == b {
                NONE
            } else if a < b {
                self.find_edge(a, b).expect("arc missing from CCH") << 1
            } else {
                (self.find_edge(b, a).expect("arc missing from CCH") << 1) | 1
            }
        })
        .collect()
    }

    /// Depth of the elimination tree (bounds query work).
    pub fn max_depth(&self) -> usize {
        let n = self.num_nodes();
        let mut depth = vec![0u32; n];
        let mut best = 0;
        for v in (0..n).rev() {
            let p = self.parent[v];
            if p != NONE {
                depth[v] = depth[p as usize] + 1;
                best = best.max(depth[v]);
            }
        }
        best as usize + 1
    }
}
