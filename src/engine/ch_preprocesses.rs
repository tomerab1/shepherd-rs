use core::f64;

use super::{graph::Graph, witness_search::local_dijkstra};

fn rank_node(graph: &Graph, node_index: usize) -> u32 {
    let in_deg = graph.bwd_edge_list[node_index].len() as u32;
    let out_deg = graph.fwd_edge_list[node_index].len() as u32;
    let node_degree = in_deg + out_deg;
    let mut num_contracted = 0u32;

    for fwd_edge in graph.get_fwd_neighbors(node_index) {
        let fwd_dest_id = fwd_edge.dest_id;

        for bwd_edge in graph.get_bwd_neighbors(node_index) {
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

            if witness_weight >= combined_weight {
                num_contracted += 1;
            }
        }
    }

    node_degree - num_contracted
}

#[cfg(test)]
mod tests {
    use crate::engine::graph::{Edge, EdgeMetadata, Node, Polyline};

    use super::*;

    //       (2)     (2)
    //     0 ---- 4 ---- 1
    // (1) |             | (1)
    //    2 ----------- 3
    //           (1)
    fn get_test_graph() -> Graph {
        Graph {
            // Forward edges
            fwd_edge_list: vec![
                vec![
                    Edge {
                        src_id: 0,
                        dest_id: 4,
                        metadata_index: 0,
                    },
                    Edge {
                        src_id: 0,
                        dest_id: 2,
                        metadata_index: 1,
                    },
                ],
                vec![Edge {
                    src_id: 1,
                    dest_id: 3,
                    metadata_index: 2,
                }],
                vec![Edge {
                    src_id: 2,
                    dest_id: 3,
                    metadata_index: 3,
                }],
                vec![Edge {
                    src_id: 3,
                    dest_id: 1,
                    metadata_index: 4,
                }],
                vec![Edge {
                    src_id: 4,
                    dest_id: 1,
                    metadata_index: 5,
                }],
            ],

            // Backward edges
            bwd_edge_list: vec![
                vec![
                    Edge {
                        src_id: 4,
                        dest_id: 0,
                        metadata_index: 0,
                    },
                    Edge {
                        src_id: 2,
                        dest_id: 0,
                        metadata_index: 1,
                    },
                ],
                vec![Edge {
                    src_id: 3,
                    dest_id: 1,
                    metadata_index: 2,
                }],
                vec![Edge {
                    src_id: 3,
                    dest_id: 2,
                    metadata_index: 3,
                }],
                vec![Edge {
                    src_id: 1,
                    dest_id: 3,
                    metadata_index: 4,
                }],
                vec![Edge {
                    src_id: 1,
                    dest_id: 4,
                    metadata_index: 5,
                }],
            ],

            nodes: vec![
                Node {
                    dense_id: 0,
                    osm_id: 100,
                    lat: 0.0,
                    lon: 0.0,
                    is_traffic_light: false,
                },
                Node {
                    dense_id: 1,
                    osm_id: 101,
                    lat: 0.0,
                    lon: 0.0,
                    is_traffic_light: false,
                },
                Node {
                    dense_id: 2,
                    osm_id: 102,
                    lat: 0.0,
                    lon: 0.0,
                    is_traffic_light: false,
                },
                Node {
                    dense_id: 3,
                    osm_id: 103,
                    lat: 0.0,
                    lon: 0.0,
                    is_traffic_light: false,
                },
                Node {
                    dense_id: 4,
                    osm_id: 104,
                    lat: 0.0,
                    lon: 0.0,
                    is_traffic_light: false,
                },
            ],

            edge_metadata: vec![
                EdgeMetadata {
                    polyline: Polyline { ids: vec![] },
                    weight: 2.0,
                    name: None,
                    speed_limit: None,
                    is_one_way: false,
                    is_roundabout: false,
                },
                EdgeMetadata {
                    polyline: Polyline { ids: vec![] },
                    weight: 1.0,
                    name: None,
                    speed_limit: None,
                    is_one_way: false,
                    is_roundabout: false,
                },
                EdgeMetadata {
                    polyline: Polyline { ids: vec![] },
                    weight: 1.0,
                    name: None,
                    speed_limit: None,
                    is_one_way: false,
                    is_roundabout: false,
                },
                EdgeMetadata {
                    polyline: Polyline { ids: vec![] },
                    weight: 1.0,
                    name: None,
                    speed_limit: None,
                    is_one_way: false,
                    is_roundabout: false,
                },
                EdgeMetadata {
                    polyline: Polyline { ids: vec![] },
                    weight: 1.0,
                    name: None,
                    speed_limit: None,
                    is_one_way: false,
                    is_roundabout: false,
                },
                EdgeMetadata {
                    polyline: Polyline { ids: vec![] },
                    weight: 2.0,
                    name: None,
                    speed_limit: None,
                    is_one_way: false,
                    is_roundabout: false,
                },
            ],
        }
    }

    #[test]
    fn test_ranking() {
        let graph = get_test_graph();

        for node in graph.get_nodes() {
            println!("{:?}", rank_node(&graph, node.dense_id));
        }
    }
}
