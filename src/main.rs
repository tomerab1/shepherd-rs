use std::{
    fs::{self, File},
    io::{Read, Write},
    time::Instant,
};

use routing_engine::engine::{
    builder::from_osmpbf, ch_preprocesses::contract_graph, ch_query, graph::Graph,
    witness_search::Dijkstra,
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

        let mut file = File::create_new("./data/graph.bin")?;
        let bytes = bincode::serialize(&overlay)?;
        file.write_all(&bytes)?;
    } else {
        let mut file = File::open("./data/graph.bin")?;
        let mut buf = vec![0u8; file.metadata().unwrap().len() as usize];

        file.read_exact(&mut buf)?;
        let graph: Graph = bincode::deserialize(&buf)?;

        let id1 = graph.nodes.iter().find(|n| n.osm_id == 2229280888).unwrap();
        let id2 = graph.nodes.iter().find(|n| n.osm_id == 2232385905).unwrap();

        let now = Instant::now();
        println!(
            "{} -> {} = {:#?}",
            id1.osm_id,
            id2.osm_id,
            ch_query::bfs(&graph, id1.dense_id, id2.dense_id)
        );
        let elapsed = now.elapsed();
        println!("Elapsed: {:.2?}", elapsed);

        let now = Instant::now();
        println!(
            "{} -> {} = {:#?}",
            id1.osm_id,
            id2.osm_id,
            ch_query::bi_dir_dijkstra(&graph, id1.dense_id, id2.dense_id)
        );
        let elapsed = now.elapsed();
        println!("Elapsed: {:.2?}", elapsed);
    }

    Ok(())
}
