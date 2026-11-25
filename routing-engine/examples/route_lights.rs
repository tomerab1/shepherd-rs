//! Diagnostics: signalised intersections a route drives through vs lights it counts.
//! Usage: route_lights <graph> <lon,lat> <lon,lat>
use routing_engine::engine::Engine;
use routing_engine::geo::Coord;
use routing_engine::graph::Graph;
use std::collections::BTreeSet;

fn main() -> anyhow::Result<()> {
    let a: Vec<String> = std::env::args().collect();
    let p = |s: &str| {
        let (lon, lat) = s.split_once(',').unwrap();
        Coord::from_deg(lat.parse().unwrap(), lon.parse().unwrap())
    };
    let e = Engine::new(Graph::load(a[1].as_ref())?);
    let g = &e.graph;
    let m = e.metric();
    let leg = e.leg(&m, &e.snap(p(&a[2])).unwrap(), &e.snap(p(&a[3])).unwrap()).unwrap();
    // Every junction the route passes through, and the intersection it belongs to.
    let mut through: BTreeSet<u32> = BTreeSet::new();
    for k in 1..leg.pieces.len() {
        let c = g.junction_cluster[g.elem_start_junction(leg.pieces[k].elem) as usize];
        if c != u32::MAX {
            through.insert(c);
        }
    }
    if let Some(watch) = a.get(4).and_then(|x| x.parse::<u32>().ok()) {
        for (k, pc) in leg.pieces.iter().enumerate() {
            let sj = g.elem_start_junction(pc.elem);
            let ej = g.elem_end_junction(pc.elem);
            let (cs, ce) = (g.junction_cluster[sj as usize], g.junction_cluster[ej as usize]);
            if cs == watch || ce == watch {
                let seg = &g.segments[pc.segment as usize];
                println!(
                    "  piece {k}: seg {} len {:.0} m from {:.2} to {:.2}; start cluster {cs}, end cluster {ce}",
                    pc.segment, seg.length_m, pc.from, pc.to
                );
            }
        }
    }
    let counted: Vec<Coord> = e.signals_on(&m, &leg.pieces);
    let counted_clusters: BTreeSet<u32> = counted
        .iter()
        .filter_map(|c| {
            (0..g.junctions.len()).find(|&j| g.junctions[j] == *c).map(|j| g.junction_cluster[j])
        })
        .collect();
    let missed: Vec<_> = through.difference(&counted_clusters).collect();
    println!(
        "  drives through {} signalised intersections, counts {} lights ({} mid-block); not counted: {:?}",
        through.len(),
        counted.len(),
        counted.len() - counted_clusters.len(),
        missed
    );
    Ok(())
}
