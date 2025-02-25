use std::{
    fs::{self, File},
    io::{Read, Write},
    time::Instant,
};

use routing_engine::engine::{
    csr::csr_graph::CSRGraph,
    export::{csv_export::CSVExport, export_provider::ExportProvider},
    preprocess::{builder::from_osmpbf, ch_preprocess::contract_graph, witness_search::Dijkstra},
    query::ch_query::BiDirDijkstra,
    visitor::{shortcut_visitor::ShortcutVisitor, visitable::Visitable},
};

fn export_node<T: ExportProvider>(exporter: T) -> T::ExportType {
    exporter.export()
}

fn main() -> anyhow::Result<()> {
    // let argv: Vec<String> = std::env::args().collect();

    if !fs::exists("./data/graph.bin")? {
        let graph = from_osmpbf(
            "/home/tomerab/VSCProjects/routing-app/routing-engine/tests/data/nz-car-only.osm.pbf",
        )?;
        println!("CREATED GRAPH");

        println!("{} {}", graph.num_edges(), graph.num_nodes());

        println!("CREATED OVERLAY");
        let mut overlay = graph.clone();
        println!("CREATED DIJKSTRA");
        let mut dijkstra = Dijkstra::new(overlay.num_nodes());

        println!("STARTING CONTRACTION");
        contract_graph(graph, &mut overlay, &mut dijkstra);

        // for node in &overlay.nodes {
        //     println!("{:?}", node);
        //     for edge in overlay.get_fwd_neighbors(node.dense_id) {
        //         println!(
        //             "{edge}\t{:?}\t{}",
        //             overlay.get_edge(*edge),
        //             overlay.get_node(overlay.get_edge(*edge).dest_id).rank
        //         );
        //     }
        //     for edge in overlay.get_bwd_neighbors(node.dense_id) {
        //         println!(
        //             "{edge}\t{:?}\t{}",
        //             overlay.get_edge(*edge),
        //             overlay.get_node(overlay.get_edge(*edge).dest_id).rank
        //         );
        //     }
        // }

        println!("FINISHED CONTRACTION");

        println!("Exporting nodes to csv");
        let exporter = CSVExport::new(&overlay, "./data/nodes.csv".to_string());
        export_node(exporter)?;
        println!("Finished exporting nodes to csv");

        let csr_graph = CSRGraph::from_preprocessed_graph(overlay);

        println!("Serializing graph to file");
        let mut file = File::create_new("./data/graph.bin")?;
        let bytes = bincode::serialize(&csr_graph)?;
        file.write_all(&bytes)?;
        println!("Finished serializing graph to file");
    } else {
        let mut file = File::open("./data/graph.bin")?;
        let mut buf = vec![0u8; file.metadata().unwrap().len() as usize];

        file.read_exact(&mut buf)?;
        let graph: CSRGraph = bincode::deserialize(&buf)?;

        let id1 = graph.nodes.iter().find(|n| n.osm_id == 2232362610).unwrap();
        let id2 = graph.nodes.iter().find(|n| n.osm_id == 2232447389).unwrap();

        let mut query = BiDirDijkstra::new(graph.nodes.len());
        query.init(id1.id, id2.id);
        let now = Instant::now();
        let query_res = query.search(&graph);
        println!("{} -> {} = {:#?}", id1.osm_id, id2.osm_id, query_res);
        let elapsed = now.elapsed();
        println!("Elapsed: {:.2?}", elapsed);

        if let Some(query_res) = query_res {
            let visitor = ShortcutVisitor::new(&graph, &query_res);
            for id in visitor.visit() {
                println!("{}", graph.nodes[id].osm_id);
            }
        } else {
            println!("Could not find path");
        }
    }

    Ok(())
}
