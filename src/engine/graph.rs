/// A ployline, represents intermediate nodes in a way.
#[derive(Debug, Clone)]
pub struct Polyline {
    // Dense index of the node.
    pub ids: Vec<usize>,
}

/// A way node.
#[derive(Debug)]
pub struct Node {
    // Dense index of the node.
    pub dense_id: usize,
    // OSM id of the node.
    pub osm_id: i64,
    // lat
    pub lat: f64,
    // lon
    pub lon: f64,
    // Is traffic light.
    pub is_traffic_light: bool,
}

/// The metadata of an edge.
#[derive(Debug, Clone)]
pub struct EdgeMetadata {
    // The intermediate nodes of the way element.
    pub polyline: Polyline,
    // The weight of the edge.
    pub weight: f64,
    // Optional name of the edge (what road/street its part of).
    pub name: Option<String>,
    // Optional maximum speed.
    pub speed_limit: Option<u8>,
    // Is one way street.
    pub is_one_way: bool,
    // Is part of a roundabout.
    pub is_roundabout: bool,
}

/// An edge
#[derive(Debug, Clone, PartialEq)]
pub struct Edge {
    // The dense id of the source node.
    pub src_id: usize,
    // The dense id of the destination node.
    pub dest_id: usize,
    // The index of the metadata of the edge in 'edge_metadata'.
    pub metadata_index: usize,
}

pub struct Graph {
    // A forward edge list, indexed by the dense id of a node.
    pub fwd_edge_list: Vec<Vec<Edge>>,
    // A backward edge list, indexed by the dense id of a node.
    pub bwd_edge_list: Vec<Vec<Edge>>,
    // Indexed by the dense id of a node.
    pub nodes: Vec<Node>,
    // Metadata for each edge.
    pub edge_metadata: Vec<EdgeMetadata>,
}

impl Graph {
    // Get the forward neighbours of a node by its dense id
    pub fn get_fwd_neighbors(&self, dense_id: usize) -> &Vec<Edge> {
        &self.fwd_edge_list[dense_id]
    }

    pub fn get_nodes(&self) -> &Vec<Node> {
        &self.nodes
    }

    // Get the forward neighbours of a node by its dense id
    pub fn get_bwd_neighbors(&self, dense_id: usize) -> &Vec<Edge> {
        &self.bwd_edge_list[dense_id]
    }

    // Gets a node by its dense id
    pub fn get_node(&self, dense_id: usize) -> &Node {
        &self.nodes[dense_id]
    }

    // Gets the metadata of an edge.
    pub fn get_edge_metadata(&self, edge: &Edge) -> &EdgeMetadata {
        &self.edge_metadata[edge.metadata_index]
    }
}
