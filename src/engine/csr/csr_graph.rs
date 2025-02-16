use rayon::iter::{IntoParallelRefIterator, ParallelIterator};
use serde::{Deserialize, Serialize};

use crate::engine::preprocess::graph::Graph;

#[derive(Debug, Serialize, Deserialize)]
pub struct CSRNode {
    pub id: usize,
    pub osm_id: i64,
    pub rank: i32,
    pub flags: u8,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct CSREdge {
    pub src: usize,
    pub dest: usize,
    pub weight: f64,
    pub name: Option<String>,
    pub prev_edge: Option<usize>,
    pub next_edge: Option<usize>,
    pub via_node: Option<usize>,
}

#[derive(Serialize, Deserialize)]
pub struct CSRGraph {
    pub cols_fwd: Vec<usize>,
    pub row_fwd_ptr: Vec<usize>,
    pub cols_bwd: Vec<usize>,
    pub row_bwd_ptr: Vec<usize>,
    pub values: Vec<CSREdge>,
    pub nodes: Vec<CSRNode>,
}

impl CSRNode {
    pub fn new(id: usize, osm_id: i64, rank: i32, flags: u8) -> Self {
        Self {
            id,
            osm_id,
            rank,
            flags,
        }
    }
}

impl CSREdge {
    pub fn new(
        src: usize,
        dest: usize,
        weight: f64,
        name: Option<String>,
        prev_edge: Option<usize>,
        next_edge: Option<usize>,
        via_node: Option<usize>,
    ) -> Self {
        Self {
            src,
            dest,
            weight,
            name,
            prev_edge,
            next_edge,
            via_node,
        }
    }
}

impl CSRGraph {
    pub fn from_preprocessed_graph(graph: Graph) -> Self {
        let mut values: Vec<CSREdge> = Vec::with_capacity(graph.num_edges());
        let mut fwd_cols = Vec::with_capacity(graph.get_num_fwd());
        let mut fwd_row_ptr = Vec::with_capacity(graph.get_num_fwd());

        fwd_row_ptr.push(0);
        for edges in &graph.fwd_edge_list {
            for id in edges {
                let edge = graph.get_edge(*id);
                let metadata = graph.get_edge_metadata(edge);
                let new_index = values.len();

                values.push(CSREdge::new(
                    edge.src_id,
                    edge.dest_id,
                    metadata.weight,
                    metadata.name.clone(),
                    edge.prev_edge,
                    edge.next_edge,
                    edge.via_node,
                ));

                fwd_cols.push(new_index);
            }

            fwd_row_ptr.push(fwd_cols.len());
        }

        let mut bwd_cols = Vec::with_capacity(graph.get_num_bwd());
        let mut bwd_row_ptr = Vec::with_capacity(graph.get_num_bwd());

        bwd_row_ptr.push(0);
        for edges in &graph.bwd_edge_list {
            for id in edges {
                let edge = graph.get_edge(*id);
                let metadata = graph.get_edge_metadata(edge);
                let new_index = values.len();

                values.push(CSREdge::new(
                    edge.src_id,
                    edge.dest_id,
                    metadata.weight,
                    metadata.name.clone(),
                    edge.prev_edge,
                    edge.next_edge,
                    edge.via_node,
                ));

                bwd_cols.push(new_index);
            }

            bwd_row_ptr.push(bwd_cols.len());
        }

        let nodes = graph
            .nodes
            .par_iter()
            .map(|node| CSRNode::new(node.dense_id, node.osm_id, node.rank, 0))
            .collect();

        Self {
            cols_bwd: bwd_cols,
            cols_fwd: fwd_cols,
            row_bwd_ptr: bwd_row_ptr,
            row_fwd_ptr: fwd_row_ptr,
            values,
            nodes,
        }
    }

    pub fn fwd_neighbors(&self, node: usize) -> impl Iterator<Item = &CSREdge> {
        let start = self.row_fwd_ptr[node];
        let end = self.row_fwd_ptr[node + 1];
        self.cols_fwd[start..end]
            .iter()
            .map(move |&edge_idx| &self.values[edge_idx])
    }

    pub fn bwd_neighbors(&self, node: usize) -> impl Iterator<Item = &CSREdge> {
        let start = self.row_bwd_ptr[node];
        let end = self.row_bwd_ptr[node + 1];
        self.cols_bwd[start..end]
            .iter()
            .map(move |&edge_idx| &self.values[edge_idx])
    }
}
