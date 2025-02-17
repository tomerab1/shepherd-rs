use std::cmp::Ordering;

use priority_queue::PriorityQueue;

use crate::engine::csr::csr_graph::CSRGraph;

#[derive(Copy, Clone, Debug)]
struct HeapItem(f32);

impl PartialEq for HeapItem {
    fn eq(&self, other: &Self) -> bool {
        (self.0 - other.0).abs() < 1e-9
    }
}

impl Eq for HeapItem {}

impl PartialOrd for HeapItem {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for HeapItem {
    fn cmp(&self, other: &Self) -> Ordering {
        other.0.partial_cmp(&self.0).unwrap()
    }
}

pub struct BiDirDijkstra {
    src: usize,
    dest: usize,
    fwd_weights: Vec<f32>,
    fwd_prev: Vec<Option<usize>>,
    bwd_weights: Vec<f32>,
    bwd_prev: Vec<Option<usize>>,
    fwd_queue: PriorityQueue<usize, HeapItem>,
    bwd_queue: PriorityQueue<usize, HeapItem>,
}

impl BiDirDijkstra {
    pub fn new(num_nodes: usize) -> Self {
        let fwd_weights = vec![f32::INFINITY; num_nodes];
        let bwd_weights = vec![f32::INFINITY; num_nodes];
        let fwd_prev = vec![None; num_nodes];
        let bwd_prev = vec![None; num_nodes];

        let fwd_queue = PriorityQueue::new();
        let bwd_queue = PriorityQueue::new();

        Self {
            src: 0,
            dest: 0,
            fwd_weights,
            bwd_weights,
            bwd_prev,
            fwd_prev,
            fwd_queue,
            bwd_queue,
        }
    }

    pub fn init(&mut self, src: usize, dest: usize) {
        self.reset();

        self.src = src;
        self.dest = dest;

        self.fwd_queue.push(self.src, HeapItem(0.0));
        self.fwd_weights[self.src] = 0.0;

        self.bwd_queue.push(self.dest, HeapItem(0.0));
        self.bwd_weights[self.dest] = 0.0;
    }

    fn reset(&mut self) {
        self.fwd_weights.fill(f32::INFINITY);
        self.bwd_weights.fill(f32::INFINITY);
        self.fwd_prev.fill(None);
        self.bwd_prev.fill(None);
        self.fwd_queue.clear();
        self.bwd_queue.clear();
    }

    fn get_path_ids(&mut self, meeting_node: Option<usize>) -> Option<Vec<usize>> {
        meeting_node.map(|node| {
            let mut path = Vec::new();
            let mut current = node;

            while let Some(prev) = self.fwd_prev[current] {
                path.push(prev);
                current = prev;
            }

            path.reverse();
            current = node;

            while let Some(prev) = self.bwd_prev[current] {
                path.push(prev);
                current = prev;
            }

            path
        })
    }

    pub fn search(&mut self, graph: &CSRGraph) -> Option<Vec<usize>> {
        let mut meeting_node = None;

        while !self.fwd_queue.is_empty() && !self.bwd_queue.is_empty() {
            if let Some((u, _)) = self.fwd_queue.pop() {
                for edge in graph.fwd_neighbors(u) {
                    let v = edge.target;
                    let weight = edge.weight;

                    if graph.nodes[v].rank < graph.nodes[u].rank {
                        continue;
                    }

                    let alt = self.fwd_weights[u] + weight;
                    if alt < self.fwd_weights[v] {
                        self.fwd_weights[v] = alt;
                        self.fwd_prev[v] = Some(u);
                        self.fwd_queue.push(v, HeapItem(alt));
                    }

                    if self.bwd_weights[v] != f32::INFINITY {
                        meeting_node = Some(v);
                        break;
                    }
                }
            }

            if let Some((u, _)) = self.bwd_queue.pop() {
                for edge in graph.bwd_neighbors(u) {
                    let v = edge.target;
                    let weight = edge.weight;

                    if graph.nodes[v].rank < graph.nodes[u].rank {
                        continue;
                    }

                    let alt = self.bwd_weights[u] + weight;
                    if alt < self.bwd_weights[v] {
                        self.bwd_weights[v] = alt;
                        self.bwd_prev[v] = Some(u);
                        self.bwd_queue.push(v, HeapItem(alt));
                    }

                    if self.fwd_weights[v] != f32::INFINITY {
                        meeting_node = Some(v);
                        break;
                    }
                }
            }

            if meeting_node.is_some() {
                break;
            }
        }

        self.get_path_ids(meeting_node)
    }
}
