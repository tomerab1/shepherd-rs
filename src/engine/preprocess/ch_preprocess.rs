use core::f64;
use std::{
    cmp::Reverse,
    sync::{Arc, Mutex},
};

use super::graph::EdgeMetadata;
use super::{graph::Graph, witness_search::Dijkstra};

use priority_queue::PriorityQueue;
use rayon::iter::{IntoParallelIterator, IntoParallelRefIterator, ParallelIterator};

pub fn contract_graph(mut graph: Graph, overlay: &mut Graph, dijkstra: &mut Dijkstra) {
    let queue = Arc::new(Mutex::new(PriorityQueue::new()));
    let mut ranks = vec![0usize; graph.num_nodes()];
    let mut order = vec![0usize; graph.num_nodes()];

    let node_ids: Vec<_> = graph.nodes.par_iter().map(|node| node.dense_id).collect();
    let num_nodes = node_ids.len();
    node_ids.into_par_iter().for_each(|id| {
        let mut dijkstra = Dijkstra::new(num_nodes);
        queue
            .lock()
            .unwrap()
            .push(id, Reverse(rank_node(overlay, &mut dijkstra, id)));
    });

    let mut queue = queue.lock().unwrap();
    let mut contraction_count = 0usize;
    while let Some((contracted_id, _)) = queue.pop() {
        println!("{} {}", overlay.get_mem_usage_str(), queue.len());
        ranks[contracted_id] = contraction_count;
        order[contraction_count] = contracted_id;

        let neighbor_rank = overlay.get_node(contracted_id).get_rank() + 1;
        contract_node(&mut graph, overlay, dijkstra, contracted_id);

        let fwd_neighbors: Vec<_> = overlay.get_fwd_neighbors(contracted_id).to_vec();
        let bwd_neighbors: Vec<_> = overlay.get_bwd_neighbors(contracted_id).to_vec();

        for edge_id in fwd_neighbors.iter().chain(bwd_neighbors.iter()) {
            let edge = overlay.get_edge(*edge_id);
            let neighbor_id = if edge.src_id == contracted_id {
                edge.dest_id
            } else {
                edge.src_id
            };
            overlay.get_node_mut(neighbor_id).raise_rank(neighbor_rank);
            let new_rank = rank_node(overlay, dijkstra, neighbor_id);
            queue.change_priority(&neighbor_id, Reverse(new_rank));
        }

        remove_edges_from_neighbors(&mut graph, contracted_id);

        contraction_count += 1;
    }
}

fn remove_edges_from_neighbors(graph: &mut Graph, contracted_id: usize) {
    let fwd_edges: Vec<_> = graph.fwd_edge_list[contracted_id].clone();
    let bwd_edges: Vec<_> = graph.bwd_edge_list[contracted_id].clone();

    for edge_idx in fwd_edges {
        let edge = graph.get_edge(edge_idx).clone();
        graph.bwd_edge_list[edge.dest_id].retain(|&e| e != edge_idx);
    }

    for edge_idx in bwd_edges {
        let edge = graph.get_edge(edge_idx).clone();
        graph.fwd_edge_list[edge.src_id].retain(|&e| e != edge_idx);
    }

    graph.fwd_edge_list[contracted_id].clear();
    graph.bwd_edge_list[contracted_id].clear();
}

fn contract_node(graph: &mut Graph, overlay: &mut Graph, dijkstra: &mut Dijkstra, node_id: usize) {
    let fwd_indices = graph.get_fwd_neighbors(node_id).clone();
    let bwd_indices = graph.get_bwd_neighbors(node_id).clone();

    for &bwd_edge_index in &bwd_indices {
        let bwd_edge = graph.get_edge(bwd_edge_index).clone();
        let w = bwd_edge.src_id;

        dijkstra.init(w, node_id);
        for &fwd_edge_index in &fwd_indices {
            let fwd_edge = graph.get_edge(fwd_edge_index);
            let v = fwd_edge.dest_id;

            if v == w || v == node_id || w == node_id {
                continue;
            }

            let weight_v_u = overlay.get_edge_metadata(&bwd_edge).weight;
            let weight_u_w = overlay.get_edge_metadata(fwd_edge).weight;
            let combined_weight = weight_v_u + weight_u_w;

            let witness_weight = dijkstra.search(graph, v, combined_weight, graph.num_edges() / 2);

            if witness_weight > combined_weight {
                add_shortcut(
                    overlay,
                    w,
                    v,
                    combined_weight,
                    bwd_edge_index,
                    fwd_edge_index,
                );
                add_shortcut(graph, w, v, combined_weight, bwd_edge_index, fwd_edge_index);
            }
        }
    }
}

fn add_shortcut(
    graph: &mut Graph,
    w: usize,
    v: usize,
    combined_weight: f64,
    left_edge_index: usize,
    right_edge_index: usize,
) {
    let shortcut_metadata = EdgeMetadata {
        weight: combined_weight,
        speed_limit: None,
        name: None,
        is_one_way: true,
        is_roundabout: false,
    };

    let metadata_index = graph.edge_metadata.len();
    graph.edge_metadata.push(shortcut_metadata);
    graph.add_shortcut_edge(w, v, metadata_index, left_edge_index, right_edge_index);
}

fn rank_node(graph: &Graph, dijkstra: &mut Dijkstra, node_index: usize) -> i32 {
    let in_deg = graph.bwd_edge_list[node_index].len() as i32;
    let out_deg = graph.fwd_edge_list[node_index].len() as i32;
    let node_degree = in_deg + out_deg;
    let mut num_contracted = 0i32;

    for bwd_edge_index in graph.get_bwd_neighbors(node_index) {
        let bwd_edge = graph.get_edge(*bwd_edge_index);
        let bwd_src_id = bwd_edge.src_id;

        dijkstra.init(bwd_src_id, node_index);
        for fwd_edge_index in graph.get_fwd_neighbors(node_index) {
            let fwd_edge = graph.get_edge(*fwd_edge_index);
            let fwd_dest_id = fwd_edge.dest_id;

            if fwd_dest_id == bwd_src_id {
                continue;
            }

            let weight_v_u = graph.get_edge_metadata(fwd_edge).weight;
            let weight_u_w = graph.get_edge_metadata(bwd_edge).weight;
            let combined_weight = weight_u_w + weight_v_u;

            let witness_weight =
                dijkstra.search(graph, fwd_dest_id, combined_weight, graph.num_nodes() / 2);
            if witness_weight > combined_weight {
                num_contracted += 1;
            }
        }
    }

    num_contracted - node_degree
}

#[cfg(test)]
mod tests {
    use crate::engine::{
        preprocess::graph::{Edge, Node},
        query::ch_query,
    };

    use super::*;

    // Test graph
    fn get_test_graph() -> Graph {
        // Node mapping:
        // 0: 1
        // 1: 3
        // 2: 4
        // 3: 5
        // 4: 6
        // 5: 7
        // 6: 8
        //
        // Graph drawing:
        //  1 -- 3 -- 4 ---> 5 ---> 6 -- 7
        //             \           /
        //              <--- 8 <---
        //
        // All edges are bidirectional.
        //
        // We create the following bidirectional edges:
        // 0 <-> 1   (represents 1 -- 3)
        // 1 <-> 2   (represents 3 -- 4)
        // 2 <-> 3   (represents 4 ---> 5)
        // 3 <-> 4   (represents 5 ---> 6)
        // 4 <-> 5   (represents 6 -- 7)
        // 2 <-> 6   (represents connection between 4 and 8)
        // 4 <-> 6   (represents connection between 6 and 8)

        let edges = vec![
            // Edge between node 0 and node 1 (1 <-> 3)
            Edge::new(0, 1, 0), // 0 -> 1, weight 10.0
            Edge::new(1, 0, 1), // 1 -> 0, weight 10.0
            // Edge between node 1 and node 2 (3 <-> 4)
            Edge::new(1, 2, 2), // 1 -> 2, weight 3.0
            Edge::new(2, 1, 3), // 2 -> 1, weight 3.0
            // Edge between node 2 and node 3 (4 <-> 5)
            Edge::new(2, 3, 4), // 2 -> 3, weight 6.0
            Edge::new(3, 2, 5), // 3 -> 2, weight 6.0
            // Edge between node 3 and node 4 (5 <-> 6)
            Edge::new(3, 4, 6), // 3 -> 4, weight 7.0
            Edge::new(4, 3, 7), // 4 -> 3, weight 7.0
            // Edge between node 4 and node 5 (6 <-> 7)
            Edge::new(4, 5, 8), // 4 -> 5, weight 8.0
            Edge::new(5, 4, 9), // 5 -> 4, weight 8.0
            // Lower branch: Edge between node 2 and node 6 (4 <-> 8)
            Edge::new(2, 6, 10), // 2 -> 6, weight 9.0
            Edge::new(6, 2, 11), // 6 -> 2, weight 9.0
            // Lower branch: Edge between node 4 and node 6 (6 <-> 8)
            Edge::new(4, 6, 12), // 4 -> 6, weight 4.0
            Edge::new(6, 4, 13), // 6 -> 4, weight 4.0
        ];

        // Build the forward edge list.
        // For each node, list the indices in `edges` for which the node is the source.
        let mut fwd_edge_list = vec![Vec::new(); 7];
        fwd_edge_list[0] = vec![0]; // node 0: 0 -> 1
        fwd_edge_list[1] = vec![1, 2]; // node 1: 1 -> 0 and 1 -> 2
        fwd_edge_list[2] = vec![3, 4, 10]; // node 2: 2 -> 1, 2 -> 3, 2 -> 6
        fwd_edge_list[3] = vec![5, 6]; // node 3: 3 -> 2, 3 -> 4
        fwd_edge_list[4] = vec![7, 8, 12]; // node 4: 4 -> 3, 4 -> 5, 4 -> 6
        fwd_edge_list[5] = vec![9]; // node 5: 5 -> 4
        fwd_edge_list[6] = vec![11, 13]; // node 6: 6 -> 2, 6 -> 4

        // Build the backward edge list.
        // For each node, list the indices in `edges` for which the node is the target.
        let mut bwd_edge_list = vec![Vec::new(); 7];
        bwd_edge_list[0] = vec![1]; // node 0: incoming edge from 1 -> 0
        bwd_edge_list[1] = vec![0, 3]; // node 1: incoming from 0 -> 1 and 2 -> 1
        bwd_edge_list[2] = vec![2, 5, 11]; // node 2: incoming from 1 -> 2, 3 -> 2, and 6 -> 2
        bwd_edge_list[3] = vec![4, 7]; // node 3: incoming from 2 -> 3 and 4 -> 3
        bwd_edge_list[4] = vec![6, 9, 13]; // node 4: incoming from 3 -> 4, 5 -> 4, and 6 -> 4
        bwd_edge_list[5] = vec![8]; // node 5: incoming from 4 -> 5
        bwd_edge_list[6] = vec![10, 12]; // node 6: incoming from 2 -> 6 and 4 -> 6

        // Create nodes. The second parameter can be used for importance, id, or any associated data.
        let nodes = vec![
            Node::new(0, 101), // represents original "1"
            Node::new(1, 103), // represents original "3"
            Node::new(2, 104), // represents original "4"
            Node::new(3, 105), // represents original "5"
            Node::new(4, 106), // represents original "6"
            Node::new(5, 107), // represents original "7"
            Node::new(6, 108), // represents original "8"
        ];

        let edge_metadata = vec![
            EdgeMetadata {
                weight: 10.0,
                name: None,
                speed_limit: None,
                is_one_way: false,
                is_roundabout: false,
            },
            EdgeMetadata {
                weight: 10.0,
                name: None,
                speed_limit: None,
                is_one_way: false,
                is_roundabout: false,
            },
            EdgeMetadata {
                weight: 3.0,
                name: None,
                speed_limit: None,
                is_one_way: false,
                is_roundabout: false,
            },
            EdgeMetadata {
                weight: 3.0,
                name: None,
                speed_limit: None,
                is_one_way: false,
                is_roundabout: false,
            },
            EdgeMetadata {
                weight: 6.0,
                name: None,
                speed_limit: None,
                is_one_way: false,
                is_roundabout: false,
            },
            EdgeMetadata {
                weight: 6.0,
                name: None,
                speed_limit: None,
                is_one_way: false,
                is_roundabout: false,
            },
            EdgeMetadata {
                weight: 7.0,
                name: None,
                speed_limit: None,
                is_one_way: false,
                is_roundabout: false,
            },
            EdgeMetadata {
                weight: 7.0,
                name: None,
                speed_limit: None,
                is_one_way: false,
                is_roundabout: false,
            },
            EdgeMetadata {
                weight: 8.0,
                name: None,
                speed_limit: None,
                is_one_way: false,
                is_roundabout: false,
            },
            EdgeMetadata {
                weight: 8.0,
                name: None,
                speed_limit: None,
                is_one_way: false,
                is_roundabout: false,
            },
            EdgeMetadata {
                weight: 9.0,
                name: None,
                speed_limit: None,
                is_one_way: false,
                is_roundabout: false,
            },
            EdgeMetadata {
                weight: 9.0,
                name: None,
                speed_limit: None,
                is_one_way: false,
                is_roundabout: false,
            },
            EdgeMetadata {
                weight: 4.0,
                name: None,
                speed_limit: None,
                is_one_way: false,
                is_roundabout: false,
            },
            EdgeMetadata {
                weight: 4.0,
                name: None,
                speed_limit: None,
                is_one_way: false,
                is_roundabout: false,
            },
        ];

        Graph {
            fwd_edge_list,
            bwd_edge_list,
            nodes,
            edges,
            edge_metadata,
        }
    }

    #[test]
    fn test_graph_contraction() {
        let graph = get_test_graph();
        let mut overlay = get_test_graph();
        let mut dijkstra = Dijkstra::new(overlay.num_nodes());

        for node in &overlay.nodes {
            println!("{}", rank_node(&graph, &mut dijkstra, node.dense_id));
        }

        contract_graph(graph, &mut overlay, &mut dijkstra);

        for node in &overlay.nodes {
            println!("{:?}", node);

            for edge in overlay.get_fwd_neighbors(node.dense_id) {
                println!("{:?}", overlay.get_edge(*edge));
            }
            println!("\n\n");
        }

        println!("{:?}", ch_query::bfs(&overlay, 1, 4));
        println!("{:?}", ch_query::bi_dir_dijkstra(&overlay, 1, 4));
    }
}
