use core::f64;
use std::{
    cmp::{Ordering, Reverse},
    collections::BinaryHeap,
};

use super::{graph::Graph, witness_search::local_dijkstra};
use crate::engine::graph::EdgeMetadata;
use rayon::iter::{IntoParallelRefIterator, ParallelIterator};

#[derive(Debug, Eq, PartialEq)]
pub struct RankedNode {
    rank: i32,
    id: usize,
}

impl Ord for RankedNode {
    fn cmp(&self, other: &Self) -> Ordering {
        self.rank.cmp(&other.rank)
    }
}

impl PartialOrd for RankedNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

pub fn contract_graph(graph: &mut Graph) {
    let mut pq = rank_nodes(graph);

    while let Some(Reverse(ranked_node)) = pq.pop() {
        let node_id = ranked_node.id;

        if graph.get_node(node_id).get_is_contracted() {
            continue;
        }

        contract_node(graph, node_id);
        update_neighbors_importance(graph, &mut pq, node_id);
    }
}

fn contract_node(graph: &mut Graph, node_id: usize) {
    let fwd_indices = graph.get_fwd_neighbors(node_id).clone();
    let bwd_indices = graph.get_bwd_neighbors(node_id).clone();

    for &fwd_edge_index in &fwd_indices {
        let fwd_edge = graph.get_edge(fwd_edge_index).clone();
        let w = fwd_edge.dest_id;

        for &bwd_edge_index in &bwd_indices {
            let bwd_edge = graph.get_edge(bwd_edge_index);
            let v = bwd_edge.src_id;

            if v == w {
                continue;
            }

            let weight_v_u = graph.get_edge_metadata(&fwd_edge).weight;
            let weight_u_w = graph.get_edge_metadata(bwd_edge).weight;
            let combined_weight = weight_v_u + weight_u_w;

            let witness_weight =
                local_dijkstra(graph, v, w, node_id, combined_weight).unwrap_or(f64::INFINITY);

            if witness_weight >= combined_weight {
                let node = graph.get_node_mut(node_id);
                node.set_is_contracted(true);
                add_shortcut(graph, v, w, combined_weight, bwd_edge_index, fwd_edge_index);
            }
        }
    }

    graph.fwd_edge_list[node_id].clear();
    graph.bwd_edge_list[node_id].clear();
}

fn add_shortcut(
    graph: &mut Graph,
    v: usize,
    w: usize,
    combined_weight: f64,
    left_edge_index: usize,
    right_edge_index: usize,
) {
    let shortcut_metadata = EdgeMetadata {
        weight: combined_weight,
        speed_limit: None,
        name: None,
        is_one_way: false,
        is_roundabout: false,
    };

    let metadata_index = graph.edge_metadata.len();
    graph.edge_metadata.push(shortcut_metadata);
    graph.add_shortcut_edge(v, w, metadata_index, left_edge_index, right_edge_index);
}

fn update_neighbors_importance(
    graph: &mut Graph,
    pq: &mut BinaryHeap<Reverse<RankedNode>>,
    node_id: usize,
) {
    let fwd_indices = graph.get_fwd_neighbors(node_id).clone();
    let bwd_indices = graph.get_bwd_neighbors(node_id).clone();

    let mut neighbors = Vec::new();
    for &edge_id in &fwd_indices {
        neighbors.push(graph.get_edge(edge_id).dest_id);
    }
    for &edge_id in &bwd_indices {
        neighbors.push(graph.get_edge(edge_id).dest_id);
    }

    // re-rank
    for neighbor_id in neighbors {
        if !graph.get_node(neighbor_id).get_is_contracted() {
            let new_rank = rank_node(graph, neighbor_id);
            pq.push(Reverse(RankedNode {
                rank: new_rank,
                id: neighbor_id,
            }));
        }
    }
}

fn rank_node(graph: &Graph, node_index: usize) -> i32 {
    let in_deg = graph.bwd_edge_list[node_index].len() as i32;
    let out_deg = graph.fwd_edge_list[node_index].len() as i32;
    let node_degree = in_deg + out_deg;
    let mut num_contracted = 0i32;

    for fwd_edge_index in graph.get_fwd_neighbors(node_index) {
        let fwd_edge = graph.get_edge(*fwd_edge_index);
        let fwd_dest_id = fwd_edge.dest_id;

        for bwd_edge_index in graph.get_bwd_neighbors(node_index) {
            let bwd_edge = graph.get_edge(*bwd_edge_index);
            let bwd_dest_id = bwd_edge.dest_id;

            if fwd_dest_id == bwd_dest_id {
                continue;
            }

            let weight_v_u = graph.get_edge_metadata(fwd_edge).weight;
            let weight_u_w = graph.get_edge_metadata(bwd_edge).weight;
            let combined_weight = weight_u_w + weight_v_u;

            let witness_weight =
                local_dijkstra(graph, fwd_dest_id, bwd_dest_id, node_index, combined_weight)
                    .unwrap_or(f64::INFINITY);

            if witness_weight > combined_weight {
                num_contracted += 1;
            }
        }
    }

    node_degree - num_contracted
}

// Ranks all the nodes in parallel and collect them into a min-heap
pub fn rank_nodes(graph: &Graph) -> BinaryHeap<Reverse<RankedNode>> {
    let ranks: BinaryHeap<Reverse<RankedNode>> = graph
        .get_nodes()
        .par_iter()
        .map(|node| {
            Reverse(RankedNode {
                rank: rank_node(graph, node.dense_id),
                id: node.dense_id,
            })
        })
        .collect();

    ranks
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::{
        builder::from_osmpbf,
        graph::{Edge, EdgeMetadata, Graph, Node},
    };

    // Test graph
    //       (2)     (2)
    //     0 ---- 4 ---- 1
    // (1) |             | (1)
    //    2 ----------- 3
    //           (1)

    fn get_test_graph() -> Graph {
        let edges = vec![
            Edge::new(0, 4, 0),
            Edge::new(0, 2, 1),
            Edge::new(1, 3, 2),
            Edge::new(2, 3, 3),
            Edge::new(3, 1, 4),
            Edge::new(4, 1, 5),
        ];

        let mut fwd_edge_list = vec![Vec::new(); 5];
        fwd_edge_list[0] = vec![0, 1];
        fwd_edge_list[1] = vec![2];
        fwd_edge_list[2] = vec![3];
        fwd_edge_list[3] = vec![4];
        fwd_edge_list[4] = vec![5];

        let mut bwd_edge_list = vec![Vec::new(); 5];
        bwd_edge_list[0] = vec![];
        bwd_edge_list[1] = vec![4, 5];
        bwd_edge_list[2] = vec![1];
        bwd_edge_list[3] = vec![2, 3];
        bwd_edge_list[4] = vec![0];

        let nodes = vec![
            Node::new(0, 100),
            Node::new(1, 101),
            Node::new(2, 102),
            Node::new(3, 103),
            Node::new(4, 104),
        ];

        let edge_metadata = vec![
            EdgeMetadata {
                weight: 2.0,
                name: None,
                speed_limit: None,
                is_one_way: false,
                is_roundabout: false,
            },
            EdgeMetadata {
                weight: 1.0,
                name: None,
                speed_limit: None,
                is_one_way: false,
                is_roundabout: false,
            },
            EdgeMetadata {
                weight: 2.0,
                name: None,
                speed_limit: None,
                is_one_way: false,
                is_roundabout: false,
            },
            EdgeMetadata {
                weight: 2.0,
                name: None,
                speed_limit: None,
                is_one_way: false,
                is_roundabout: false,
            },
            EdgeMetadata {
                weight: 1.0,
                name: None,
                speed_limit: None,
                is_one_way: false,
                is_roundabout: false,
            },
            EdgeMetadata {
                weight: 2.0,
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
    fn test_ranking() {
        let graph = get_test_graph();

        for node in graph.get_nodes() {
            let r = rank_node(&graph, node.dense_id);
            println!("Node={} rank={}", node.dense_id, r);
        }

        let all_ranks = rank_nodes(&graph);
        for std::cmp::Reverse(rn) in all_ranks {
            println!("Node={} final_rank={}", rn.id, rn.rank);
        }
    }

    #[test]
    fn test_graph_contraction() {
        let mut graph = get_test_graph();
        contract_graph(&mut graph);

        for node in graph.nodes.iter() {
            println!("{:?}", node);
        }

        for edge in graph.edges.iter() {
            println!("{:?}", edge);
        }

        for metadata in graph.edge_metadata.iter() {
            println!("{:?}", metadata);
        }
    }

    #[test]
    fn test() {
        let mut graph = from_osmpbf("tests/data/nz-car-only.osm.pbf").unwrap();

        contract_graph(&mut graph);
    }
}
