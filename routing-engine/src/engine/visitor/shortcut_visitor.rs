use std::collections::HashSet;

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

    fn push_node(out: &mut Vec<usize>, node: usize) {
        if out.last() != Some(&node) {
            out.push(node);
        }
    }

    fn visit_shortcut(
        graph: &CSRGraph,
        edge: &CSREdgeCold,
        out: &mut Vec<usize>,
        is_fwd: bool,
        visited: &mut HashSet<usize>,
    ) {
        if is_fwd {
            if visited.contains(&edge.from_node) {
                return;
            }
        } else {
            if visited.contains(&edge.to_node) {
                return;
            }
        }

        if edge.via_node.is_none() {
            if is_fwd {
                ShortcutVisitor::push_node(out, edge.from_node);
                visited.insert(edge.from_node);
            } else {
                ShortcutVisitor::push_node(out, edge.to_node);
                visited.insert(edge.to_node);
            }
        }
        if let (Some(prev_edge_id), Some(next_edge_id)) = (edge.prev_edge, edge.next_edge) {
            let prev_edge = graph.get_fwd_edge_cold(prev_edge_id);
            let next_edge = graph.get_bwd_edge_cold(next_edge_id);

            if is_fwd {
                ShortcutVisitor::visit_shortcut(graph, prev_edge, out, is_fwd, visited);
                ShortcutVisitor::visit_shortcut(graph, next_edge, out, is_fwd, visited);
            } else {
                ShortcutVisitor::visit_shortcut(graph, next_edge, out, is_fwd, visited);
                ShortcutVisitor::visit_shortcut(graph, prev_edge, out, is_fwd, visited);
            }
        }
    }
}

impl<'a> Visitable for ShortcutVisitor<'a> {
    type Output = Vec<usize>;

    fn visit(&self) -> Self::Output {
        let mut nodes = Vec::new();
        let mut visited = HashSet::new();

        for QueryResult { edge_id, is_fwd } in self.packed_path {
            let edge = &self.graph.get_fwd_edge_cold(*edge_id);
            ShortcutVisitor::visit_shortcut(self.graph, edge, &mut nodes, *is_fwd, &mut visited);
        }

        nodes
    }
}
