use core::f64;
use std::{
    cmp::{Ordering, Reverse},
    collections::{BinaryHeap, HashMap},
};

use super::graph::Graph;

#[derive(Debug, Copy, Clone, PartialEq)]
struct MinHeapItem(usize, f64);

impl Eq for MinHeapItem {}

impl Ord for MinHeapItem {
    fn cmp(&self, other: &Self) -> Ordering {
        self.1
            .partial_cmp(&other.1)
            .unwrap_or(Ordering::Equal)
            .reverse()
    }
}

impl PartialOrd for MinHeapItem {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

// A local dijkstra search, for the witness search phase of ch.
pub fn local_dijkstra(
    graph: &Graph,
    src_node: usize,
    dest_node: usize,
    exclude_node: usize,
    limit_weight: f64,
) -> Option<f64> {
    let mut queue = BinaryHeap::new();
    let mut dist_map: HashMap<usize, f64> = HashMap::new();

    dist_map.insert(src_node, 0.0);
    queue.push(Reverse(MinHeapItem(src_node, 0.0)));

    while let Some(Reverse(MinHeapItem(curr_id, curr_dist))) = queue.pop() {
        // Check if the curr_id node is in the map, if not itll return inf, else itll return
        // the distance from the source, if the distance is greater than the curr distance, then we can continue.
        if curr_dist > dist_map.get(&curr_id).copied().unwrap_or(f64::INFINITY) {
            continue;
        }

        // if the current distance is greater than limit weight, we can assume that the shortest path
        // to dest_node will also result in a larger weight than limit_weight, thus we stop the search.
        if curr_dist >= limit_weight {
            return None;
        }

        // If we reached the dest node we can stop are return the distance.
        if curr_id == dest_node {
            return Some(curr_dist);
        }

        // Iterate over each neighbor, skip the excluded node, update the node if there is a shorter path and insert it to the priority queue.
        for neighbor in graph.get_fwd_neighbors(curr_id) {
            let neighbor_id = neighbor.dest_id;

            if neighbor_id == exclude_node {
                continue;
            }

            let alt = curr_dist + graph.get_edge_metadata(neighbor).weight;
            if alt < dist_map.get(&neighbor_id).copied().unwrap_or(f64::INFINITY) {
                dist_map.insert(neighbor_id, alt);
                queue.push(Reverse(MinHeapItem(neighbor_id, alt)));
            }
        }
    }

    None
}

#[cfg(test)]
mod tests {
    use crate::engine::graph::{Edge, EdgeMetadata};

    use super::*;

    //       (2)     (2)
    //     0 ---- 4 ---- 1
    // (1) |             | (1)
    //    2 ----------- 3
    //           (1)
    fn get_test_graph() -> Graph {
        Graph {
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
            bwd_edge_list: vec![],
            nodes: vec![],
            edge_metadata: vec![
                EdgeMetadata {
                    polyline: None,
                    weight: 2.0,
                    name: None,
                    speed_limit: None,
                    is_one_way: false,
                    is_roundabout: false,
                },
                EdgeMetadata {
                    polyline: None,
                    weight: 1.0,
                    name: None,
                    speed_limit: None,
                    is_one_way: false,
                    is_roundabout: false,
                },
                EdgeMetadata {
                    polyline: None,
                    weight: 1.0,
                    name: None,
                    speed_limit: None,
                    is_one_way: false,
                    is_roundabout: false,
                },
                EdgeMetadata {
                    polyline: None,
                    weight: 1.0,
                    name: None,
                    speed_limit: None,
                    is_one_way: false,
                    is_roundabout: false,
                },
                EdgeMetadata {
                    polyline: None,
                    weight: 1.0,
                    name: None,
                    speed_limit: None,
                    is_one_way: false,
                    is_roundabout: false,
                },
                EdgeMetadata {
                    polyline: None,
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
    fn test_local_dijkstra() {
        let graph = get_test_graph();
        let weight = local_dijkstra(&graph, 0, 1, 4, 4.0);
        assert_eq!(weight, Some(3.0));
    }
}
