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
        for neighbor_index in graph.get_fwd_neighbors(curr_id) {
            let neighbor_edge = graph.get_edge(*neighbor_index);
            let neighbor_id = neighbor_edge.dest_id;

            if neighbor_id == exclude_node {
                continue;
            }

            let alt = curr_dist + graph.get_edge_metadata(neighbor_edge).weight;
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
    use super::*;
    use crate::engine::graph::{Edge, EdgeMetadata};

    //
    //       (2)     (2)
    //     0 ---- 4 ---- 1
    //  (1)|             |(1)
    //    2 ----------- 3
    //         (1)
    //
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

        fwd_edge_list[0].push(0);
        fwd_edge_list[0].push(1);
        fwd_edge_list[1].push(2);
        fwd_edge_list[2].push(3);
        fwd_edge_list[3].push(4);
        fwd_edge_list[4].push(5);

        let mut bwd_edge_list = vec![Vec::new(); 5];

        bwd_edge_list[4].push(0);
        bwd_edge_list[2].push(1);
        bwd_edge_list[3].push(2);
        bwd_edge_list[3].push(3);
        bwd_edge_list[1].push(4);
        bwd_edge_list[1].push(5);

        let edge_metadata = vec![
            EdgeMetadata {
                weight: 2.0, // (0->4)
                name: None,
                speed_limit: None,
                is_one_way: false,
                is_roundabout: false,
            },
            EdgeMetadata {
                weight: 1.0, // (0->2)
                name: None,
                speed_limit: None,
                is_one_way: false,
                is_roundabout: false,
            },
            EdgeMetadata {
                weight: 1.0, // (1->3)
                name: None,
                speed_limit: None,
                is_one_way: false,
                is_roundabout: false,
            },
            EdgeMetadata {
                weight: 1.0, // (2->3)
                name: None,
                speed_limit: None,
                is_one_way: false,
                is_roundabout: false,
            },
            EdgeMetadata {
                weight: 1.0, // (3->1)
                name: None,
                speed_limit: None,
                is_one_way: false,
                is_roundabout: false,
            },
            EdgeMetadata {
                weight: 2.0, // (4->1)
                name: None,
                speed_limit: None,
                is_one_way: false,
                is_roundabout: false,
            },
        ];

        Graph {
            fwd_edge_list,
            bwd_edge_list,
            edges,
            edge_metadata,
            nodes: vec![],
        }
    }

    #[test]
    fn test_local_dijkstra() {
        let graph = get_test_graph();
        let weight = local_dijkstra(&graph, 0, 1, 4, 4.0);

        assert_eq!(weight, Some(3.0));
    }
}
