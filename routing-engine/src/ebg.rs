//! Edge-based graph (EBG).
//!
//! Every EBG node is a *directed segment* ("drive segment s from its `from` to its `to` junction"
//! or the reverse). An EBG arc a -> b is a legal turn: a ends at the junction where b starts.
//! Turn restrictions simply remove arcs, and turn costs live on arcs, which is why routing on
//! this graph handles both exactly.

use crate::geo::{bearing, fast_dist, turn_angle, Coord};
use crate::network::{Network, Segment};
use crate::osm::{profile::Profile, OsmData, RestrictionKind};
use std::collections::{HashMap, HashSet};

/// A directed segment packed as `segment << 1 | backward`.
pub type DSeg = u32;

#[inline]
pub fn dseg(seg: u32, backward: bool) -> DSeg {
    (seg << 1) | backward as u32
}
#[inline]
pub fn dseg_seg(d: DSeg) -> u32 {
    d >> 1
}
#[inline]
pub fn dseg_forward(d: DSeg) -> bool {
    d & 1 == 0
}

/// EBG node flag: belongs to the big strongly connected component (safe to snap to).
pub const NODE_ROUTABLE: u8 = 1;

pub const NO_NODE: u32 = u32::MAX;

#[derive(Debug, Default, Clone)]
pub struct Ebg {
    /// EBG node -> directed segment.
    pub elem_dseg: Vec<DSeg>,
    pub elem_flags: Vec<u8>,
    /// CSR of outgoing arcs.
    pub first_out: Vec<u32>,
    pub head: Vec<u32>,
    /// Turn cost of each arc in deciseconds (excludes the travel time of the target segment).
    pub turn: Vec<u16>,
}

impl Ebg {
    pub fn num_nodes(&self) -> usize {
        self.elem_dseg.len()
    }
    pub fn num_arcs(&self) -> usize {
        self.head.len()
    }
    #[inline]
    pub fn out(&self, n: u32) -> std::ops::Range<usize> {
        self.first_out[n as usize] as usize..self.first_out[n as usize + 1] as usize
    }

    /// Relabels nodes so that `new_id = perm[old_id]`.
    pub fn permute(&self, perm: &[u32]) -> Ebg {
        let n = self.num_nodes();
        let mut inv = vec![0u32; n];
        for (old, &new) in perm.iter().enumerate() {
            inv[new as usize] = old as u32;
        }
        let mut out = Ebg {
            elem_dseg: Vec::with_capacity(n),
            elem_flags: Vec::with_capacity(n),
            first_out: Vec::with_capacity(n + 1),
            head: Vec::with_capacity(self.num_arcs()),
            turn: Vec::with_capacity(self.num_arcs()),
        };
        out.first_out.push(0);
        for &old in &inv {
            out.elem_dseg.push(self.elem_dseg[old as usize]);
            out.elem_flags.push(self.elem_flags[old as usize]);
            for a in self.out(old) {
                out.head.push(perm[self.head[a] as usize]);
                out.turn.push(self.turn[a]);
            }
            out.first_out.push(out.head.len() as u32);
        }
        out
    }
}

/// Bearing at the start or end of a directed segment, measured over at least ~10 m so short
/// geometry wiggles near junctions don't produce bogus turn angles.
pub fn dseg_bearing(geom: &[Coord], forward: bool, at_end: bool) -> f64 {
    let n = geom.len();
    let pt = |i: usize| if forward { geom[i] } else { geom[n - 1 - i] };
    if at_end {
        let end = pt(n - 1);
        let mut i = n - 2;
        while i > 0 && fast_dist(pt(i), end) < 10.0 {
            i -= 1;
        }
        bearing(pt(i), end)
    } else {
        let start = pt(0);
        let mut i = 1;
        while i < n - 1 && fast_dist(start, pt(i)) < 10.0 {
            i += 1;
        }
        bearing(start, pt(i))
    }
}

/// Result of building the EBG: the graph plus the segment -> EBG node lookup.
pub struct EbgBuild {
    pub ebg: Ebg,
    /// For each segment, the EBG node of (forward, backward), `NO_NODE` if not traversable.
    pub seg_elems: Vec<[u32; 2]>,
    pub restrictions_applied: usize,
}

pub fn build(net: &Network, osm: &OsmData, profile: Profile) -> EbgBuild {
    let nseg = net.segments.len();
    let nj = net.junctions.len();

    // 1. Allocate EBG nodes.
    let mut seg_elems = vec![[NO_NODE; 2]; nseg];
    let mut elem_dseg = Vec::with_capacity(nseg * 2);
    for (s, seg) in net.segments.iter().enumerate() {
        for (k, fwd) in [(0usize, true), (1usize, false)] {
            if seg.allows(fwd) {
                seg_elems[s][k] = elem_dseg.len() as u32;
                elem_dseg.push(dseg(s as u32, !fwd));
            }
        }
    }
    let n = elem_dseg.len();

    // 2. Junction -> directed segments leaving it (CSR).
    let leaving_junction = |seg: &Segment, fwd: bool| if fwd { seg.from } else { seg.to };
    let arriving_junction = |seg: &Segment, fwd: bool| if fwd { seg.to } else { seg.from };
    let mut first_leave = vec![0u32; nj + 1];
    for &d in &elem_dseg {
        let seg = &net.segments[dseg_seg(d) as usize];
        first_leave[leaving_junction(seg, dseg_forward(d)) as usize + 1] += 1;
    }
    for j in 0..nj {
        first_leave[j + 1] += first_leave[j];
    }
    let mut leave = vec![0u32; n];
    let mut fill = first_leave.clone();
    for (node, &d) in elem_dseg.iter().enumerate() {
        let seg = &net.segments[dseg_seg(d) as usize];
        let j = leaving_junction(seg, dseg_forward(d)) as usize;
        leave[fill[j] as usize] = node as u32;
        fill[j] += 1;
    }
    // Number of distinct segments touching each junction (for dead-end U-turns).
    let mut degree = vec![0u32; nj];
    for seg in &net.segments {
        degree[seg.from as usize] += 1;
        if seg.to != seg.from {
            degree[seg.to as usize] += 1;
        }
    }

    // 3. Turn restrictions, resolved to EBG node pairs.
    let junction_of: HashMap<i64, u32> = net
        .junction_osm
        .iter()
        .enumerate()
        .map(|(i, &o)| (o, i as u32))
        .collect();
    let mut arriving_at: HashMap<u32, Vec<u32>> = HashMap::new();
    let via_junctions: HashSet<u32> = osm
        .restrictions
        .iter()
        .filter_map(|r| r.via_node().and_then(|v| junction_of.get(&v).copied()))
        .collect();
    for (node, &d) in elem_dseg.iter().enumerate() {
        let seg = &net.segments[dseg_seg(d) as usize];
        let j = arriving_junction(seg, dseg_forward(d));
        if via_junctions.contains(&j) {
            arriving_at.entry(j).or_default().push(node as u32);
        }
    }
    let mut forbidden: HashSet<(u32, u32)> = HashSet::new();
    let mut only: HashMap<u32, Vec<u32>> = HashMap::new();
    let mut applied = 0usize;
    for r in &osm.restrictions {
        let Some(&j) = r.via_node().and_then(|v| junction_of.get(&v)) else { continue };
        let from: Vec<u32> = arriving_at
            .get(&j)
            .map(|v| {
                v.iter()
                    .copied()
                    .filter(|&a| net.segment_way[dseg_seg(elem_dseg[a as usize]) as usize] == r.from_way)
                    .collect()
            })
            .unwrap_or_default();
        let to: Vec<u32> = leave[first_leave[j as usize] as usize..first_leave[j as usize + 1] as usize]
            .iter()
            .copied()
            .filter(|&b| net.segment_way[dseg_seg(elem_dseg[b as usize]) as usize] == r.to_way)
            .collect();
        if from.is_empty() || to.is_empty() {
            continue;
        }
        applied += 1;
        for &a in &from {
            match r.kind {
                RestrictionKind::No => {
                    for &b in &to {
                        forbidden.insert((a, b));
                    }
                }
                RestrictionKind::Only => only.entry(a).or_default().extend(to.iter().copied()),
            }
        }
    }

    // 4. Arcs, as per-node lists first so via-way restrictions can duplicate nodes.
    let (u_turn_anywhere, u_turn_s) = profile.u_turn();
    let mut out: Vec<Vec<(u32, u16)>> = Vec::with_capacity(n);
    for (a, &da) in elem_dseg.iter().enumerate() {
        let mut list = Vec::new();
        let a = a as u32;
        let sa = dseg_seg(da);
        let seg_a = &net.segments[sa as usize];
        let fa = dseg_forward(da);
        let j = arriving_junction(seg_a, fa) as usize;
        let in_bearing = dseg_bearing(net.geometry_of(seg_a), fa, true);
        let dead_end = degree[j] <= 1;
        for &b in &leave[first_leave[j] as usize..first_leave[j + 1] as usize] {
            let db = elem_dseg[b as usize];
            let u_turn = dseg_seg(db) == sa;
            if u_turn && !dead_end && !u_turn_anywhere {
                continue;
            }
            if forbidden.contains(&(a, b)) {
                continue;
            }
            if let Some(allowed) = only.get(&a) {
                if !allowed.contains(&b) {
                    continue;
                }
            }
            let seg_b = &net.segments[dseg_seg(db) as usize];
            let out_bearing = dseg_bearing(net.geometry_of(seg_b), dseg_forward(db), false);
            let secs = if u_turn {
                u_turn_s
            } else {
                profile.turn_penalty_s(turn_angle(in_bearing, out_bearing))
            };
            list.push((b, ((secs * 10.0).round() as u32).min(u16::MAX as u32) as u16));
        }
        out.push(list);
    }

    // 5. Via-way restrictions by node duplication.
    let mut elem_dseg = elem_dseg;
    for r in &osm.restrictions {
        if let crate::osm::Via::Ways(vias) = &r.via {
            if apply_via_way(net, &seg_elems, &mut elem_dseg, &mut out, r.from_way, vias, r.to_way, r.kind) {
                applied += 1;
            }
        }
    }

    let n = elem_dseg.len();
    let mut first_out = Vec::with_capacity(n + 1);
    let mut head = Vec::with_capacity(n * 3);
    let mut turn = Vec::with_capacity(n * 3);
    first_out.push(0u32);
    for list in &out {
        for &(b, t) in list {
            head.push(b);
            turn.push(t);
        }
        first_out.push(head.len() as u32);
    }
    drop(out);

    let mut ebg = Ebg {
        elem_dseg,
        elem_flags: vec![0; n],
        first_out,
        head,
        turn,
    };
    mark_routable(&mut ebg);
    EbgBuild {
        ebg,
        seg_elems,
        restrictions_applied: applied,
    }
}

/// Applies a via-way restriction `from -> via ways -> to` by duplicating the via chain for
/// each entry from `from`: the entry arc is redirected into private copies of the via
/// segments, and only the copy of the last via segment has its turn onto `to` removed
/// (`no_*`) or made the only turn (`only_*`). Copies keep the segment of the original, so
/// geometry, durations and instructions are unchanged. Returns false if the restriction does
/// not match the network (members missing, not connected, or the chain is not drivable).
#[allow(clippy::too_many_arguments)]
fn apply_via_way(
    net: &Network,
    seg_elems: &[[u32; 2]],
    elem_dseg: &mut Vec<DSeg>,
    out: &mut Vec<Vec<(u32, u16)>>,
    from_way: i64,
    vias: &[i64],
    to_way: i64,
    kind: RestrictionKind,
) -> bool {
    // Way -> segments index, built once per network (restrictions are applied in a loop).
    thread_local! {
        static INDEX: std::cell::RefCell<Option<(usize, HashMap<i64, Vec<u32>>)>> =
            const { std::cell::RefCell::new(None) };
    }
    let lookup = |w: i64| -> Vec<u32> {
        INDEX.with(|ix| {
            let mut ix = ix.borrow_mut();
            let key = net.segments.as_ptr() as usize ^ net.segments.len();
            if ix.as_ref().is_none_or(|(k, _)| *k != key) {
                let mut m: HashMap<i64, Vec<u32>> = HashMap::new();
                for (s, &way) in net.segment_way.iter().enumerate() {
                    m.entry(way).or_default().push(s as u32);
                }
                *ix = Some((key, m));
            }
            ix.as_ref().unwrap().1.get(&w).cloned().unwrap_or_default()
        })
    };
    let from_segs = lookup(from_way);
    let to_segs = lookup(to_way);
    let via_segs: Vec<u32> = vias.iter().flat_map(|&w| lookup(w)).collect();
    if from_segs.is_empty() || to_segs.is_empty() || via_segs.is_empty() {
        return false;
    }
    let ends = |s: u32| {
        let seg = &net.segments[s as usize];
        [seg.from, seg.to]
    };
    let via_junctions: HashSet<u32> = via_segs.iter().flat_map(|&s| ends(s)).collect();
    // Entry junction: where the from-way touches the via ways; exit: where the to-way does.
    let Some(j1) = from_segs.iter().flat_map(|&s| ends(s)).find(|j| via_junctions.contains(j)) else {
        return false;
    };
    let Some(j2) = to_segs
        .iter()
        .flat_map(|&s| ends(s))
        .find(|j| via_junctions.contains(j) && *j != j1)
    else {
        return false;
    };

    // Walk the via segments from j1 to j2 (BFS over junctions), collecting directed segments.
    let mut prev: HashMap<u32, (u32, u32, bool)> = HashMap::new(); // junction -> (from junction, seg, forward)
    let mut queue = std::collections::VecDeque::from([j1]);
    let mut seen: HashSet<u32> = HashSet::from([j1]);
    while let Some(j) = queue.pop_front() {
        if j == j2 {
            break;
        }
        for &s in &via_segs {
            let seg = &net.segments[s as usize];
            for (fwd, a, b) in [(true, seg.from, seg.to), (false, seg.to, seg.from)] {
                if a == j && seg.allows(fwd) && seen.insert(b) {
                    prev.insert(b, (a, s, fwd));
                    queue.push_back(b);
                }
            }
        }
    }
    if !seen.contains(&j2) {
        return false;
    }
    let mut chain: Vec<DSeg> = Vec::new();
    let mut j = j2;
    while j != j1 {
        let (a, s, fwd) = prev[&j];
        chain.push(dseg(s, !fwd));
        j = a;
    }
    chain.reverse();

    // Directed from-segments arriving at j1, and to-segments leaving j2.
    let entries: Vec<u32> = from_segs
        .iter()
        .flat_map(|&s| {
            let seg = &net.segments[s as usize];
            [(true, seg.to), (false, seg.from)]
                .into_iter()
                .filter(move |&(_, arrive)| arrive == j1)
                .map(move |(fwd, _)| seg_elems[s as usize][(!fwd) as usize])
        })
        .filter(|&x| x != NO_NODE)
        .collect();
    let exits: HashSet<DSeg> = to_segs
        .iter()
        .flat_map(|&s| {
            let seg = &net.segments[s as usize];
            [(true, seg.from), (false, seg.to)]
                .into_iter()
                .filter(move |&(fwd, leave)| leave == j2 && seg.allows(fwd))
                .map(move |(fwd, _)| dseg(s, !fwd))
        })
        .collect();
    if entries.is_empty() || exits.is_empty() {
        return false;
    }

    let mut any = false;
    for &a in &entries {
        let mut cur = a;
        let mut ok = true;
        for &d in &chain {
            // The arc from `cur` into this via segment (original or an earlier copy).
            let Some(k) = out[cur as usize].iter().position(|&(b, _)| elem_dseg[b as usize] == d) else {
                ok = false;
                break;
            };
            let (target, t) = out[cur as usize][k];
            let copy = elem_dseg.len() as u32;
            elem_dseg.push(d);
            let arcs = out[target as usize].clone();
            out.push(arcs);
            out[cur as usize][k] = (copy, t);
            cur = copy;
        }
        if !ok {
            continue;
        }
        let last = &mut out[cur as usize];
        match kind {
            RestrictionKind::No => last.retain(|&(b, _)| !exits.contains(&elem_dseg[b as usize])),
            RestrictionKind::Only => last.retain(|&(b, _)| exits.contains(&elem_dseg[b as usize])),
        }
        any = true;
    }
    any
}

/// Marks nodes of the largest strongly connected component (and any SCC with at least
/// 1000 nodes) as routable. Snapping ignores the rest, so a start point on an isolated
/// parking lot or a broken one-way stub doesn't produce "no route".
fn mark_routable(g: &mut Ebg) {
    let comp = scc(g);
    let ncomp = comp.iter().copied().max().map_or(0, |m| m as usize + 1);
    let mut size = vec![0u32; ncomp];
    for &c in &comp {
        size[c as usize] += 1;
    }
    let largest = size.iter().copied().max().unwrap_or(0);
    let threshold = largest.min(1000);
    for (i, &c) in comp.iter().enumerate() {
        if size[c as usize] >= threshold {
            g.elem_flags[i] |= NODE_ROUTABLE;
        }
    }
}

/// Iterative Tarjan SCC. Returns a component id per node.
pub fn scc(g: &Ebg) -> Vec<u32> {
    scc_csr(g.num_nodes(), &g.first_out, &g.head)
}

/// Iterative Tarjan SCC over a CSR graph. Returns a component id per node.
pub fn scc_csr(n: usize, first_out: &[u32], head: &[u32]) -> Vec<u32> {
    let out = |v: u32| first_out[v as usize] as usize..first_out[v as usize + 1] as usize;
    const UNSEEN: u32 = u32::MAX;
    let mut index = vec![UNSEEN; n];
    let mut low = vec![0u32; n];
    let mut on_stack = vec![false; n];
    let mut comp = vec![UNSEEN; n];
    let mut stack: Vec<u32> = Vec::new();
    let mut call: Vec<(u32, usize)> = Vec::new();
    let mut next_index = 0u32;
    let mut next_comp = 0u32;

    for root in 0..n as u32 {
        if index[root as usize] != UNSEEN {
            continue;
        }
        call.push((root, out(root).start));
        index[root as usize] = next_index;
        low[root as usize] = next_index;
        next_index += 1;
        stack.push(root);
        on_stack[root as usize] = true;

        while let Some(&mut (v, ref mut it)) = call.last_mut() {
            let end = out(v).end;
            if *it < end {
                let w = head[*it];
                *it += 1;
                if index[w as usize] == UNSEEN {
                    index[w as usize] = next_index;
                    low[w as usize] = next_index;
                    next_index += 1;
                    stack.push(w);
                    on_stack[w as usize] = true;
                    call.push((w, out(w).start));
                } else if on_stack[w as usize] {
                    low[v as usize] = low[v as usize].min(index[w as usize]);
                }
            } else {
                call.pop();
                if let Some(&(parent, _)) = call.last() {
                    low[parent as usize] = low[parent as usize].min(low[v as usize]);
                }
                if low[v as usize] == index[v as usize] {
                    loop {
                        let w = stack.pop().unwrap();
                        on_stack[w as usize] = false;
                        comp[w as usize] = next_comp;
                        if w == v {
                            break;
                        }
                    }
                    next_comp += 1;
                }
            }
        }
    }
    comp
}

/// A node-based graph for profiles without turn restrictions or turn costs (walking):
/// nodes are junctions, arcs are directed segments. Several times smaller than the
/// edge-based graph, and its CCH is smaller still.
pub struct NodeGraph {
    pub num_nodes: usize,
    pub first_out: Vec<u32>,
    pub head: Vec<u32>,
    /// Directed segment of each arc (arcs are the routing "elements").
    pub arc_dseg: Vec<DSeg>,
    pub arc_flags: Vec<u8>,
    /// Per segment: arc of [forward, backward], `NO_NODE` if not allowed.
    pub seg_arcs: Vec<[u32; 2]>,
    /// Network junction -> graph node.
    pub junction_node: Vec<u32>,
}

impl NodeGraph {
    /// Builds the graph with nodes numbered by `rank[junction]`.
    pub fn build(net: &Network, rank: &[u32]) -> NodeGraph {
        let n = net.junctions.len();
        let mut arcs: Vec<(u32, u32, DSeg)> = Vec::with_capacity(net.segments.len() * 2);
        for (s, seg) in net.segments.iter().enumerate() {
            if seg.allows(true) {
                arcs.push((rank[seg.from as usize], rank[seg.to as usize], dseg(s as u32, false)));
            }
            if seg.allows(false) {
                arcs.push((rank[seg.to as usize], rank[seg.from as usize], dseg(s as u32, true)));
            }
        }
        arcs.sort_unstable();
        let mut first_out = vec![0u32; n + 1];
        for &(t, _, _) in &arcs {
            first_out[t as usize + 1] += 1;
        }
        for i in 0..n {
            first_out[i + 1] += first_out[i];
        }
        let mut seg_arcs = vec![[NO_NODE; 2]; net.segments.len()];
        for (i, &(_, _, d)) in arcs.iter().enumerate() {
            seg_arcs[dseg_seg(d) as usize][(!dseg_forward(d)) as usize] = i as u32;
        }
        let head: Vec<u32> = arcs.iter().map(|a| a.1).collect();
        let arc_dseg: Vec<DSeg> = arcs.iter().map(|a| a.2).collect();

        // Snap only onto arcs inside a big strongly connected component.
        let comp = scc_csr(n, &first_out, &head);
        let ncomp = comp.iter().copied().max().map_or(0, |m| m as usize + 1);
        let mut size = vec![0u32; ncomp];
        for &c in &comp {
            size[c as usize] += 1;
        }
        let threshold = size.iter().copied().max().unwrap_or(0).min(1000);
        let mut arc_flags = vec![0u8; arcs.len()];
        for (i, &(t, h, _)) in arcs.iter().enumerate() {
            if comp[t as usize] == comp[h as usize] && size[comp[t as usize] as usize] >= threshold {
                arc_flags[i] = NODE_ROUTABLE;
            }
        }
        NodeGraph {
            num_nodes: n,
            first_out,
            head,
            arc_dseg,
            arc_flags,
            seg_arcs,
            junction_node: rank.to_vec(),
        }
    }
}
