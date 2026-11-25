//! CCH customization: turns input arc weights into weights for every CCH edge.
//!
//! Basic customization processes "lower triangles": for nodes u < v < w forming a triangle,
//! the path v -> u -> w may be shorter than the edge v -> w. Nodes are processed level by level;
//! within a level each node only writes its own upward edges and only reads edges of nodes on
//! lower levels, so a level is processed in parallel without locks.

use super::{Topology, INF, NONE};
use rayon::prelude::*;

/// Weights of one metric (e.g. "free flow" or "with current traffic").
pub struct Metric {
    /// Weight of each CCH edge (lo, hi) in the lo -> hi direction.
    pub up: Vec<u32>,
    /// Weight of each CCH edge (lo, hi) in the hi -> lo direction.
    pub down: Vec<u32>,
}

#[derive(Clone, Copy)]
struct Shared(*mut u32);
// SAFETY: see module docs; each node writes a disjoint range of edges.
unsafe impl Send for Shared {}
unsafe impl Sync for Shared {}

#[inline]
fn add(a: u32, b: u32) -> u32 {
    (a + b).min(INF)
}

/// Levels with fewer nodes than this are processed on the calling thread.
const PAR_LEVEL_MIN: usize = 256;

/// `arc_w(i)` is the weight of input arc `i` (computed on the fly, not stored).
pub fn customize(topo: &Topology, arc_to_cch: &[u32], arc_w: impl Fn(usize) -> u32) -> Metric {
    let m = topo.num_edges();
    let mut up = vec![INF; m];
    let mut down = vec![INF; m];
    for (i, &code) in arc_to_cch.iter().enumerate() {
        if code == NONE {
            continue;
        }
        let e = (code >> 1) as usize;
        let w = arc_w(i).min(INF);
        if code & 1 == 0 {
            up[e] = up[e].min(w);
        } else {
            down[e] = down[e].min(w);
        }
    }

    let up_p = Shared(up.as_mut_ptr());
    let down_p = Shared(down.as_mut_ptr());
    let nlevels = topo.level_first.len() - 1;
    for l in 0..nlevels {
        let nodes = &topo.level_nodes[topo.level_first[l] as usize..topo.level_first[l + 1] as usize];
        let run = |&v: &u32| {
            // SAFETY: node v writes only edges in topo.up(v); reads edges of lower-level nodes,
            // which are final and not written by anyone in this level.
            unsafe { process_node(topo, v, up_p, down_p) }
        };
        if nodes.len() >= PAR_LEVEL_MIN {
            nodes.par_iter().for_each(run);
        } else {
            nodes.iter().for_each(run);
        }
    }
    Metric { up, down }
}

#[inline]
unsafe fn process_node(topo: &Topology, v: u32, up: Shared, down: Shared) {
    let (up, down) = (up.0, down.0);
    let vr = topo.up(v);
    if vr.is_empty() {
        return;
    }
    let v_heads = &topo.up_head[vr.clone()];
    for k in topo.down(v) {
        let u = topo.down_tail[k];
        let e_uv = topo.down_edge[k] as usize;
        let ur = topo.up(u);
        let (d_uv, u_uv) = (*down.add(e_uv), *up.add(e_uv));
        if d_uv >= INF && u_uv >= INF {
            continue;
        }
        // up(u) is sorted; everything after (u, v) is > v.
        let mut i = e_uv + 1;
        let mut j = 0usize;
        while i < ur.end && j < v_heads.len() {
            let wu = topo.up_head[i];
            let wv = v_heads[j];
            if wu < wv {
                i += 1;
            } else if wv < wu {
                j += 1;
            } else {
                let e_vw = vr.start + j;
                let e_uw = i;
                let cand_up = add(d_uv, *up.add(e_uw));
                let cand_down = add(*down.add(e_uw), u_uv);
                let pu = up.add(e_vw);
                if cand_up < *pu {
                    *pu = cand_up;
                }
                let pd = down.add(e_vw);
                if cand_down < *pd {
                    *pd = cand_down;
                }
                i += 1;
                j += 1;
            }
        }
    }
}
