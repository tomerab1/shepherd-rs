use routing_engine::engine::{builder::from_osmpbf, ch_preprocesses::contract_graph};

fn main() {
    let argv: Vec<String> = std::env::args().collect();

    let mut graph = from_osmpbf(&argv[1]).unwrap();

    contract_graph(&mut graph);

    println!("{}", graph.get_mem_usage_str());

    // for node in graph.nodes {
    //     println!("{:?}", node);
    // }

    for edge in graph.edges {
        println!("{:?}", edge);
    }

    // for metadata in graph.edge_metadata {
    //     println!("{:?}", metadata);
    // }
}
