use std::cmp::Ordering;

use priority_queue::PriorityQueue;

use crate::engine::preprocess::graph::Graph;

#[derive(Copy, Clone, Debug)]
struct HeapItem(f64);

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

pub fn bi_dir_dijkstra(overlay: &Graph, src: usize, dest: usize) -> Option<Vec<usize>> {
    let mut forward_dist = vec![f64::INFINITY; overlay.num_nodes()];
    let mut backward_dist = vec![f64::INFINITY; overlay.num_nodes()];
    let mut forward_prev = vec![None; overlay.num_nodes()];
    let mut backward_prev = vec![None; overlay.num_nodes()];

    let mut forward_queue = PriorityQueue::new();
    let mut backward_queue = PriorityQueue::new();

    forward_dist[src] = 0.0;
    backward_dist[dest] = 0.0;
    forward_queue.push(src, HeapItem(0.0));
    backward_queue.push(dest, HeapItem(0.0));

    let mut meeting_node = None;

    while !forward_queue.is_empty() && !backward_queue.is_empty() {
        if let Some((u, _)) = forward_queue.pop() {
            for id in overlay.get_fwd_neighbors(u) {
                let edge = overlay.get_edge(*id);
                let v = edge.dest_id;
                let weight = overlay.get_edge_metadata(edge).weight;

                let node = overlay.get_node(v);

                if node.rank < overlay.get_node(u).rank {
                    continue;
                }

                let alt = forward_dist[u] + weight;
                if alt < forward_dist[v] {
                    forward_dist[v] = alt;
                    forward_prev[v] = Some(u);
                    forward_queue.push(v, HeapItem(alt));
                }

                if backward_dist[v] != f64::INFINITY {
                    meeting_node = Some(v);
                    break;
                }
            }
        }

        if let Some((u, _)) = backward_queue.pop() {
            for id in overlay.get_bwd_neighbors(u) {
                let edge = overlay.get_edge(*id);
                let v = edge.src_id;
                let weight = overlay.get_edge_metadata(edge).weight;

                let node = overlay.get_node(v);

                if node.rank < overlay.get_node(u).rank {
                    continue;
                }

                let alt = backward_dist[u] + weight;
                if alt < backward_dist[v] {
                    backward_dist[v] = alt;
                    backward_prev[v] = Some(u);
                    backward_queue.push(v, HeapItem(alt));
                }

                if forward_dist[v] != f64::INFINITY {
                    meeting_node = Some(v);
                    break;
                }
            }
        }

        if meeting_node.is_some() {
            break;
        }
    }

    meeting_node.map(|node| {
        let mut path = Vec::new();
        let mut current = node;

        while let Some(prev) = forward_prev[current] {
            path.push(prev);
            current = prev;
        }

        path.reverse();
        current = node;

        while let Some(prev) = backward_prev[current] {
            path.push(prev);
            current = prev;
        }

        path
    })
}

use std::collections::VecDeque;

pub fn bfs(graph: &Graph, start: usize, end: usize) -> Option<Vec<(usize, i32)>> {
    if start == end {
        return Some(vec![(start, 0)]);
    }

    let n = graph.num_nodes();
    let mut visited = vec![false; n];
    let mut parent = vec![None; n];

    let mut queue = VecDeque::new();
    visited[start] = true;
    queue.push_back(start);

    while let Some(current_node) = queue.pop_front() {
        for &edge_id in &graph.fwd_edge_list[current_node] {
            let edge = &graph.edges[edge_id];
            let neighbor = edge.dest_id;

            if !visited[neighbor] {
                visited[neighbor] = true;
                parent[neighbor] = Some(current_node);
                queue.push_back(neighbor);

                if neighbor == end {
                    let rank = graph.get_node(end).get_rank();
                    let mut path = vec![(end, rank)];
                    let mut p = current_node;

                    while p != start {
                        let rank = graph.get_node(p).get_rank();
                        path.push((p, rank));
                        p = parent[p].unwrap();
                    }

                    let rank = graph.get_node(start).get_rank();
                    path.push((start, rank));
                    path.reverse();

                    return Some(path);
                }
            }
        }
    }

    None
}
