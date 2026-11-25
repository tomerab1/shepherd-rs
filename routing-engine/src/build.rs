//! Preprocessing pipeline: OSM -> network -> edge-based graph -> CCH order and topology -> file.

use crate::cch::{order, Topology};
use crate::ebg::{self, NODE_ROUTABLE, NO_NODE};
use crate::geo::LocalProjection;
use crate::graph::{Graph, Meta};
use crate::network::Network;
use crate::osm::{self, profile::Profile, OsmData};
use crate::snap::build_grid;
use std::time::Instant;
use tracing::info;

pub fn preprocess_file(input: &str, source_name: &str, profile: Profile) -> anyhow::Result<Graph> {
    let t = Instant::now();
    let osm = osm::read_pbf(input, profile)?;
    info!(
        ways = osm.ways.len(),
        nodes = osm.nodes.len(),
        restrictions = osm.restrictions.len(),
        "read OSM in {:.1?}",
        t.elapsed()
    );
    Ok(build_graph(&osm, source_name, profile))
}

/// The routing graph a profile uses, before it is assembled into a [`Graph`].
struct Routing {
    node_based: bool,
    num_nodes: usize,
    elem_dseg: Vec<u32>,
    elem_flags: Vec<u8>,
    first_out: Vec<u32>,
    head: Vec<u32>,
    turn: Vec<u16>,
    /// Flattened per segment: element of [forward, backward].
    seg_elems: Vec<u32>,
    junction_node: Vec<u32>,
    restrictions_applied: usize,
}

fn order_nodes(n: usize, xy: Vec<[f32; 2]>, edges: impl Iterator<Item = (u32, u32)>) -> Vec<u32> {
    let t = Instant::now();
    let ordered = order::nested_dissection(order::SubGraph::from_edges(n, xy, edges));
    let mut rank = vec![0u32; n];
    for (r, &v) in ordered.iter().enumerate() {
        rank[v as usize] = r as u32;
    }
    info!("nested dissection order in {:.1?}", t.elapsed());
    rank
}

/// Edge-based graph (driving): nodes are directed segments, arcs are legal turns.
fn edge_based(net: &Network, osm: &OsmData, profile: Profile, proj: &LocalProjection) -> Routing {
    let t = Instant::now();
    let eb = ebg::build(net, osm, profile);
    let g0 = eb.ebg;
    info!(
        nodes = g0.num_nodes(),
        arcs = g0.num_arcs(),
        restrictions = eb.restrictions_applied,
        "built edge-based graph in {:.1?}",
        t.elapsed()
    );
    // Coordinates for the ordering: midpoint of each directed segment's geometry.
    let xy: Vec<[f32; 2]> = g0
        .elem_dseg
        .iter()
        .map(|&d| {
            let seg = &net.segments[ebg::dseg_seg(d) as usize];
            let geom = net.geometry_of(seg);
            let (x, y) = proj.project(geom[geom.len() / 2]);
            [x as f32, y as f32]
        })
        .collect();
    let n = g0.num_nodes();
    let rank = order_nodes(
        n,
        xy,
        (0..n as u32).flat_map(|a| g0.out(a).map(move |i| (a, i))).map(|(a, i)| (a, g0.head[i])),
    );
    let g = g0.permute(&rank);
    drop(g0);
    let seg_elems: Vec<u32> = eb
        .seg_elems
        .iter()
        .flat_map(|p| p.map(|x| if x == NO_NODE { NO_NODE } else { rank[x as usize] }))
        .collect();
    Routing {
        node_based: false,
        num_nodes: n,
        elem_dseg: g.elem_dseg,
        elem_flags: g.elem_flags,
        first_out: g.first_out,
        head: g.head,
        turn: g.turn,
        seg_elems,
        junction_node: Vec::new(),
        restrictions_applied: eb.restrictions_applied,
    }
}

/// Node-based graph (walking): nodes are junctions, arcs are directed segments.
fn node_based(net: &Network, proj: &LocalProjection) -> Routing {
    let n = net.junctions.len();
    let xy: Vec<[f32; 2]> = net
        .junctions
        .iter()
        .map(|&c| {
            let (x, y) = proj.project(c);
            [x as f32, y as f32]
        })
        .collect();
    let rank = order_nodes(
        n,
        xy,
        net.segments
            .iter()
            .filter(|s| s.flags & (crate::network::SEG_FORWARD | crate::network::SEG_BACKWARD) != 0)
            .map(|s| (s.from, s.to)),
    );
    let g = ebg::NodeGraph::build(net, &rank);
    info!(nodes = g.num_nodes, arcs = g.head.len(), "built node-based graph");
    Routing {
        node_based: true,
        num_nodes: g.num_nodes,
        elem_dseg: g.arc_dseg,
        elem_flags: g.arc_flags,
        first_out: g.first_out,
        head: g.head,
        turn: Vec::new(),
        seg_elems: g.seg_arcs.iter().flat_map(|p| *p).collect(),
        junction_node: g.junction_node,
        restrictions_applied: 0,
    }
}

pub fn build_graph(osm: &OsmData, source_name: &str, profile: Profile) -> Graph {
    let t = Instant::now();
    let net = Network::build(osm, profile);
    info!(
        junctions = net.junctions.len(),
        segments = net.segments.len(),
        geometry_points = net.geometry.len(),
        "built network in {:.1?}",
        t.elapsed()
    );

    let (minlat, maxlat) = net
        .junctions
        .iter()
        .fold((f64::MAX, f64::MIN), |(a, b), c| (a.min(c.lat()), b.max(c.lat())));
    let ref_lat = if net.junctions.is_empty() { 0.0 } else { (minlat + maxlat) / 2.0 };
    let proj = LocalProjection::new(ref_lat);

    let g = if profile.node_based() {
        node_based(&net, &proj)
    } else {
        edge_based(&net, osm, profile, &proj)
    };

    let t = Instant::now();
    let n = g.num_nodes;
    let arcs = || {
        (0..n as u32).flat_map(|a| (g.first_out[a as usize]..g.first_out[a as usize + 1]).map(move |i| (a, i)))
            .map(|(a, i)| (a, g.head[i as usize]))
    };
    let topo = Topology::build(n, arcs());
    let arc_to_cch = topo.map_arcs(arcs());
    let depth = topo.max_depth();
    info!(
        cch_edges = topo.num_edges(),
        elimination_tree_depth = depth,
        levels = topo.level_first.len() - 1,
        "built CCH topology in {:.1?}",
        t.elapsed()
    );

    let seg_elems = &g.seg_elems;
    let routable = |s: usize| {
        seg_elems[s * 2..s * 2 + 2]
            .iter()
            .any(|&x| x != NO_NODE && g.elem_flags[x as usize] & NODE_ROUTABLE != 0)
    };
    let grid = build_grid(&net.segments, &net.geometry, routable, &proj);

    let mut way_order: Vec<u32> = (0..net.segments.len() as u32).collect();
    way_order.sort_by_key(|&s| net.segment_way[s as usize]);

    let (mut bbox, mut any) = ([f64::MAX, f64::MAX, f64::MIN, f64::MIN], false);
    for c in &net.junctions {
        any = true;
        bbox = [bbox[0].min(c.lon()), bbox[1].min(c.lat()), bbox[2].max(c.lon()), bbox[3].max(c.lat())];
    }
    if !any {
        bbox = [0.0; 4];
    }

    let meta = Meta {
        source: source_name.to_string(),
        profile: profile.name().to_string(),
        node_based: g.node_based,
        built_at_unix: std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or(0),
        ref_lat,
        grid_origin: grid.origin,
        grid_cell_m: grid.cell_m,
        grid_nx: grid.nx,
        grid_ny: grid.ny,
        bbox,
        num_junctions: net.junctions.len(),
        num_segments: net.segments.len(),
        num_ebg_nodes: n,
        num_ebg_arcs: g.head.len(),
        num_cch_edges: topo.num_edges(),
        num_signals: net.signals.len(),
        elimination_tree_depth: depth,
        restrictions_applied: g.restrictions_applied,
        restrictions_skipped: osm.skipped_restrictions,
    };

    Graph {
        meta,
        junctions: net.junctions.into(),
        segments: net.segments.into(),
        geometry: net.geometry.into(),
        name_bytes: net.names.bytes.into(),
        name_offsets: net.names.offsets.into(),
        segment_way: net.segment_way.into(),
        way_order: way_order.into(),
        seg_elems: g.seg_elems.into(),
        elem_dseg: g.elem_dseg.into(),
        elem_flags: g.elem_flags.into(),
        first_out: g.first_out.into(),
        head: g.head.into(),
        turn: g.turn.into(),
        junction_node: g.junction_node.into(),
        topo,
        arc_to_cch: arc_to_cch.into(),
        grid_first: grid.first.into(),
        grid_items: grid.items.into(),
        signals: net.signals.into(),
    }
}
