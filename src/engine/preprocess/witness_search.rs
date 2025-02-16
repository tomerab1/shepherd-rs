use core::f64;
use std::cmp::Ordering;

use priority_queue::PriorityQueue;

use super::graph::Graph;

#[derive(Debug, Copy, Clone, PartialEq)]
struct HeapItem(f64);

impl Eq for HeapItem {}

impl Ord for HeapItem {
    fn cmp(&self, other: &Self) -> Ordering {
        other.0.partial_cmp(&self.0).unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for HeapItem {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

pub struct Dijkstra {
    src: usize,
    ignore: usize,
    weights: Vec<f64>,
    queue: PriorityQueue<usize, HeapItem>,
}

impl Dijkstra {
    pub fn new(num_nodes: usize) -> Self {
        Self {
            src: 0,
            ignore: 0,
            weights: vec![f64::INFINITY; num_nodes],
            queue: PriorityQueue::new(),
        }
    }

    pub fn init(&mut self, src: usize, ignore: usize) {
        self.reset();

        self.src = src;
        self.ignore = ignore;
        self.queue.push(self.src, HeapItem(0.0));
        self.weights[self.src] = 0.0;
    }

    fn reset(&mut self) {
        self.weights.fill(f64::INFINITY);
        self.queue.clear();
    }

    pub fn search(
        &mut self,
        graph: &Graph,
        dest: usize,
        limit_weight: f64,
        max_hops: usize,
    ) -> f64 {
        let mut num_hops = 0;
        while let Some((curr_id, _)) = self.queue.pop() {
            if num_hops == max_hops {
                break;
            }

            if self.weights[dest] <= limit_weight {
                return self.weights[dest];
            }

            for id in graph.get_fwd_neighbors(curr_id) {
                let neighbor_edge = graph.get_edge(*id);
                let neighbor_id = neighbor_edge.dest_id;

                if neighbor_id == self.ignore {
                    continue;
                }

                let weight = self.weights[curr_id] + graph.get_edge_metadata(neighbor_edge).weight;
                if weight == f64::INFINITY {
                    continue;
                }
                let adj_weight = self.weights[neighbor_id];
                if weight < adj_weight || adj_weight == f64::INFINITY {
                    self.weights[neighbor_id] = weight;
                    self.queue.push(neighbor_id, HeapItem(weight));
                }
            }

            num_hops += 1;

            if curr_id == dest {
                return self.weights[dest];
            }
        }

        self.weights[dest]
    }
}

#[cfg(test)]
mod tests {
    use crate::engine::preprocess::graph::{Edge, EdgeMetadata, Node};

    use super::*;

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

    #[test]
    fn test_local_dijkstra() {
        let graph = get_test_graph();
        // let weight = local_dijkstra(&graph, 0, 4, 3, 21.0, 100);

        // assert_eq!(weight, Some(18.0));
    }
}
