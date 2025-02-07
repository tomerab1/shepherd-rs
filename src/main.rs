use routing_engine::engine::{builder::from_osmpbf, ch_preprocesses::contract_graph};

fn main() {
    let mut graph =
        from_osmpbf("/home/tomerab/VSCProjects/routing-engine/tests/data/nz-car-only.pbf.osm")
            .unwrap();

    contract_graph(&mut graph);

    println!("{}", graph.get_mem_usage_str());
    println!("{:?}", graph.edges);
}
