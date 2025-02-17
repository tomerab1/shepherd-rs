use std::{
    fs::{self, File},
    io::{Read, Write},
    time::Instant,
};

use routing_engine::engine::{
    csr::csr_graph::CSRGraph,
    preprocess::{builder::from_osmpbf, ch_preprocess::contract_graph, witness_search::Dijkstra},
    query::ch_query::BiDirDijkstra,
};

fn main() -> anyhow::Result<()> {
    // let argv: Vec<String> = std::env::args().collect();

    if !fs::exists("./data/graph.bin")? {
        let graph =
            from_osmpbf("/home/tomerab/VSCProjects/routing-engine/tests/data/nz-car-only.osm.pbf")?;
        println!("CREATED GRAPH");

        println!("{} {}", graph.num_edges(), graph.num_nodes());

        println!("CREATED OVERLAY");
        let mut overlay = graph.clone();
        println!("CREATED DIJKSTRA");
        let mut dijkstra = Dijkstra::new(overlay.num_nodes());

        println!("STARTING CONTRACTION");
        contract_graph(graph, &mut overlay, &mut dijkstra);
        println!("FINISHED CONTRACTION");

        let csr_graph = CSRGraph::from_preprocessed_graph(overlay);

        let mut file = File::create_new("./data/graph.bin")?;
        let bytes = bincode::serialize(&csr_graph)?;
        file.write_all(&bytes)?;
    } else {
        let mut file = File::open("./data/graph.bin")?;
        let mut buf = vec![0u8; file.metadata().unwrap().len() as usize];

        file.read_exact(&mut buf)?;
        let graph: CSRGraph = bincode::deserialize(&buf)?;

        let id1 = graph.nodes.iter().find(|n| n.osm_id == 2229280888).unwrap();
        let id2 = graph.nodes.iter().find(|n| n.osm_id == 2232385899).unwrap();

        let mut query = BiDirDijkstra::new(graph.nodes.len());
        query.init(id1.id, id2.id);
        let now = Instant::now();
        let query_res = query.search(&graph);
        println!("{} -> {} = {:#?}", id1.osm_id, id2.osm_id, query_res);
        let elapsed = now.elapsed();
        println!("Elapsed: {:.2?}", elapsed);

        for id in query_res.unwrap() {
            println!("{}", graph.nodes[id].osm_id);
        }
    }

    Ok(())
}
