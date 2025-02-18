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
pub struct CSREdgeHot {
    pub target: usize,
    pub weight: f32,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct CSREdgeCold {
    pub name: Option<String>,
    pub from_node: usize,
    pub to_node: usize,
    pub via_node: Option<usize>,
}

#[derive(Serialize, Deserialize)]
pub struct CSRGraph {
    pub cols_fwd: Vec<usize>,
    pub row_fwd_ptr: Vec<usize>,
    pub cols_bwd: Vec<usize>,
    pub row_bwd_ptr: Vec<usize>,
    pub values_hot: Vec<CSREdgeHot>,
    pub values_cold: Vec<CSREdgeCold>,
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

impl CSREdgeHot {
    pub fn new(target: usize, weight: f32) -> Self {
        Self { target, weight }
    }
}

impl CSREdgeCold {
    pub fn new(
        name: Option<String>,
        from_node: usize,
        to_node: usize,
        via_node: Option<usize>,
    ) -> Self {
        Self {
            name,
            from_node,
            to_node,
            via_node,
        }
    }
}

impl CSRGraph {
    pub fn from_preprocessed_graph(graph: Graph) -> Self {
        let mut values_hot: Vec<CSREdgeHot> = Vec::with_capacity(graph.num_edges());
        let mut values_cold: Vec<CSREdgeCold> = Vec::with_capacity(graph.num_edges());
        let mut fwd_cols = Vec::with_capacity(graph.get_num_fwd());
        let mut fwd_row_ptr = Vec::with_capacity(graph.get_num_fwd());

        fwd_row_ptr.push(0);
        for edges in &graph.fwd_edge_list {
            for id in edges {
                let edge = graph.get_edge(*id);
                let metadata = graph.get_edge_metadata(edge);
                let new_index = values_hot.len();

                values_hot.push(CSREdgeHot::new(edge.dest_id, metadata.weight));

                values_cold.push(CSREdgeCold::new(
                    metadata.name.clone(),
                    edge.src_id,
                    edge.dest_id,
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
                let new_index = values_hot.len();

                values_hot.push(CSREdgeHot::new(edge.src_id, metadata.weight));

                values_cold.push(CSREdgeCold::new(
                    metadata.name.clone(),
                    edge.src_id,
                    edge.dest_id,
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
            values_hot,
            values_cold,
            nodes,
        }
    }

    pub fn fwd_neighbors_cold(&self, node: usize) -> impl Iterator<Item = (usize, &CSREdgeCold)> {
        let start = self.row_fwd_ptr[node];
        let end = self.row_fwd_ptr[node + 1];
        self.cols_fwd[start..end]
            .iter()
            .map(|&edge_idx| (edge_idx, &self.values_cold[edge_idx]))
    }

    pub fn bwd_neighbors_cold(&self, node: usize) -> impl Iterator<Item = (usize, &CSREdgeCold)> {
        let start = self.row_bwd_ptr[node];
        let end = self.row_bwd_ptr[node + 1];
        self.cols_bwd[start..end]
            .iter()
            .map(|&edge_idx| (edge_idx, &self.values_cold[edge_idx]))
    }

    pub fn fwd_neighbors(&self, node: usize) -> impl Iterator<Item = (usize, &CSREdgeHot)> {
        let start = self.row_fwd_ptr[node];
        let end = self.row_fwd_ptr[node + 1];
        self.cols_fwd[start..end]
            .iter()
            .map(|&edge_idx| (edge_idx, &self.values_hot[edge_idx]))
    }

    pub fn bwd_neighbors(&self, node: usize) -> impl Iterator<Item = (usize, &CSREdgeHot)> {
        let start = self.row_bwd_ptr[node];
        let end = self.row_bwd_ptr[node + 1];
        self.cols_bwd[start..end]
            .iter()
            .map(|&edge_idx| (edge_idx, &self.values_hot[edge_idx]))
    }
}
