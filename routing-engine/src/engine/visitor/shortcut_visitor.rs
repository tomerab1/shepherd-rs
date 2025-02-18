use crate::engine::{
    csr::csr_graph::{CSREdgeCold, CSRGraph},
    query::ch_query::QueryResult,
};

use super::visitable::Visitable;

/// A shortcut visiotr, unpacks the contracted path to restore the original path.
pub struct ShortcutVisitor<'a> {
    /// Reference to the graph
    pub graph: &'a CSRGraph,
    /// Reference to the query result.
    pub packed_path: &'a [QueryResult],
}

impl<'a> ShortcutVisitor<'a> {
    pub fn new(graph: &'a CSRGraph, packed_path: &'a [QueryResult]) -> Self {
        Self { graph, packed_path }
    }

    /// Helper function for visiting a shortcut.
    pub fn visit_shortcut(
        graph: &CSRGraph,
        edge: &CSREdgeCold,
        out: &mut Vec<usize>,
        is_fwd: bool,
    ) {
        // Get the via node.
        // A shortcut in a path A -> B -> C is of the form A -> C and the additional info of the 'via' node 'B'
        if let Some(via) = edge.via_node {
            if is_fwd {
                if out.last() != Some(&edge.from_node) {
                    out.push(edge.from_node);
                }

                // Find the edge A -> via
                let u_to_via =
                    graph
                        .fwd_neighbors_cold(edge.from_node)
                        .find_map(|(edge_id, cold)| {
                            if cold.to_node == via {
                                Some(&graph.values_cold[edge_id])
                            } else {
                                None
                            }
                        });

                if let Some(first_edge) = u_to_via {
                    // visit the edge A -> via to unpack nested shortcuts.
                    ShortcutVisitor::visit_shortcut(graph, first_edge, out, is_fwd);
                } else {
                    todo!("handle case");
                }

                // Visit the edge via -> C
                let via_to_v = graph.fwd_neighbors_cold(via).find_map(|(edge_id, cold)| {
                    if cold.to_node == edge.to_node {
                        Some(&graph.values_cold[edge_id])
                    } else {
                        None
                    }
                });

                if let Some(second_edge) = via_to_v {
                    // visit the edge via -> C to unpack nested shortcuts.
                    ShortcutVisitor::visit_shortcut(graph, second_edge, out, is_fwd);
                } else {
                    todo!("handle case");
                }

                if out.last() != Some(&edge.to_node) {
                    out.push(edge.to_node);
                }
            } else {
                if out.last() != Some(&edge.to_node) {
                    out.push(edge.to_node);
                }

                // Get the backward edge that goes into via from 'C' (C -> via)
                let v_to_via =
                    graph
                        .bwd_neighbors_cold(edge.to_node)
                        .find_map(|(edge_id, cold)| {
                            if cold.from_node == via {
                                Some(&graph.values_cold[edge_id])
                            } else {
                                None
                            }
                        });

                if let Some(first_edge) = v_to_via {
                    // visit the edge C -> via to unpack nested shortcuts.
                    ShortcutVisitor::visit_shortcut(graph, first_edge, out, is_fwd);
                } else {
                    todo!("handle case");
                }

                // Get the backward edge that goes into 'A' (via -> A)
                let via_to_u = graph.bwd_neighbors_cold(via).find_map(|(edge_id, cold)| {
                    if cold.from_node == edge.from_node {
                        Some(&graph.values_cold[edge_id])
                    } else {
                        None
                    }
                });

                if let Some(second_edge) = via_to_u {
                    // visit the edge via -> A to unpack nested shortcuts.
                    ShortcutVisitor::visit_shortcut(graph, second_edge, out, is_fwd);
                } else {
                    eprintln!("Failed to find backward edge from via_node: {}", via);
                }

                if out.last() != Some(&edge.from_node) {
                    out.push(edge.from_node);
                }
            }
        }
    }
}

impl<'a> Visitable for ShortcutVisitor<'a> {
    // vec of node osm ids
    type Output = Vec<usize>;

    fn visit(&self) -> Self::Output {
        let mut nodes = Vec::new();

        for QueryResult { edge_id, is_fwd } in self.packed_path {
            let edge = &self.graph.values_cold[*edge_id];
            ShortcutVisitor::visit_shortcut(self.graph, edge, &mut nodes, *is_fwd);
        }

        nodes
    }
}
