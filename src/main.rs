use routing_engine::engine::{builder::from_osmpbf, ch_preprocesses::rank_nodes};

fn main() {
    let graph =
        from_osmpbf("/home/tomerab/VSCProjects/routing-engine/data/il-car-only.osm.pbf").unwrap();

    println!("{}", graph.get_mem_usage_str());

    _ = rank_nodes(&graph);
}
