use core::f64;
use std::cmp::Reverse;

use super::{graph::Graph, witness_search::Dijkstra};
use crate::engine::graph::EdgeMetadata;

use priority_queue::PriorityQueue;

pub fn contract_graph(mut graph: Graph, overlay: &mut Graph, dijkstra: &mut Dijkstra) {
    let mut queue = PriorityQueue::new();
    let mut ranks = vec![0usize; graph.num_nodes()];
    let mut order = vec![0usize; graph.num_nodes()];

    let node_ids: Vec<_> = graph.nodes.iter().map(|node| node.dense_id).collect();
    for id in node_ids {
        queue.push(id, Reverse(rank_node(overlay, dijkstra, id)));
    }

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

            let witness_weight = dijkstra.search(graph, v, combined_weight, usize::MAX);

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
        is_one_way: true, // Ensure shortcut is directed
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

            let witness_weight = dijkstra.search(graph, fwd_dest_id, combined_weight, usize::MAX);
            if witness_weight > combined_weight {
                num_contracted += 1;
            }
        }
    }

    num_contracted - node_degree
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::{
        ch_query,
        graph::{Edge, EdgeMetadata, Graph, Node},
    };

    // Test graph
    //           10                 3
    // (p) 0 <---------> (v) 1 <---------> (r) 2
    //                6  |                 | 5
    //                  (q) 3 <---------> (w) 4
    //                             5

    fn get_test_graph() -> Graph {
        let edges = vec![
            Edge::new(0, 1, 0), // 0 -> 1
            Edge::new(1, 0, 1), // 1 -> 0
            Edge::new(1, 2, 2), // 1 -> 2
            Edge::new(2, 1, 3), // 2 -> 1
            Edge::new(1, 3, 4), // 1 -> 3
            Edge::new(3, 1, 5), // 3 -> 1
            Edge::new(3, 4, 6), // 3 -> 4
            Edge::new(4, 3, 7), // 4 -> 3
            Edge::new(2, 4, 8), // 2 -> 4
            Edge::new(4, 2, 9), // 4 -> 2
        ];

        let mut fwd_edge_list = vec![Vec::new(); 5];
        fwd_edge_list[0] = vec![0];
        fwd_edge_list[1] = vec![1, 2, 4];
        fwd_edge_list[2] = vec![3, 8];
        fwd_edge_list[3] = vec![5, 6];
        fwd_edge_list[4] = vec![7, 9];

        let mut bwd_edge_list = vec![Vec::new(); 5];
        bwd_edge_list[0] = vec![1];
        bwd_edge_list[1] = vec![0, 3, 5];
        bwd_edge_list[2] = vec![2, 9];
        bwd_edge_list[3] = vec![4, 7];
        bwd_edge_list[4] = vec![6, 8];

        let nodes = vec![
            Node::new(0, 100),
            Node::new(1, 101),
            Node::new(2, 102),
            Node::new(3, 103),
            Node::new(4, 104),
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
                weight: 5.0,
                name: None,
                speed_limit: None,
                is_one_way: false,
                is_roundabout: false,
            },
            EdgeMetadata {
                weight: 5.0,
                name: None,
                speed_limit: None,
                is_one_way: false,
                is_roundabout: false,
            },
            EdgeMetadata {
                weight: 5.0,
                name: None,
                speed_limit: None,
                is_one_way: false,
                is_roundabout: false,
            },
            EdgeMetadata {
                weight: 5.0,
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

    // Test graph
    //           10                 3
    // (p) 0 <---------> (v) 1 <---------> (r) 2
    //                6  |                 | 5
    //                  (q) 3 <---------> (w) 4
    //                             5

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
                if overlay.get_edge(*edge).is_shortcut() {
                    println!("{:?}", overlay.get_edge(*edge));
                }
            }
            println!("\n\n");
        }

        println!("{:?}", ch_query::bfs(&overlay, 1, 4));
        println!("{:?}", ch_query::bi_dir_dijkstra(&overlay, 4, 1));
    }
}
