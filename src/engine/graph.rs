use humansize::{format_size, DECIMAL};

/// A way node.
#[derive(Debug)]
pub struct Node {
    // Dense index of the node.
    pub dense_id: usize,
    // OSM id of the node.
    pub osm_id: i64,
    // The rank of the node
    pub rank: i32,
    // Flag for identifying if the node was contracted or not
    pub is_contracted: bool,
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
    // Flag for identifying if the edge is a shortcut or not.
    pub is_shortcut: bool,
    // Dense index of the previous edge.
    pub prev_edge: Option<usize>,
    // Dense index of the next edge.
    pub next_edge: Option<usize>,
}

pub struct Graph {
    // A forward edge list, indexed by the dense id of a node.
    pub fwd_edge_list: Vec<Vec<usize>>,
    // A backward edge list, indexed by the dense id of a node.
    pub bwd_edge_list: Vec<Vec<usize>>,
    // Indexed by the dense id of a node.
    pub nodes: Vec<Node>,
    // Indexed by the dense id of an edge
    pub edges: Vec<Edge>,
    // Metadata for each edge.
    pub edge_metadata: Vec<EdgeMetadata>,
}

impl Graph {
    // Get the forward neighbours of a node by its dense id
    pub fn get_fwd_neighbors(&self, dense_id: usize) -> &Vec<usize> {
        &self.fwd_edge_list[dense_id]
    }

    pub fn get_nodes(&self) -> &Vec<Node> {
        &self.nodes
    }

    // Get the forward neighbours of a node by its dense id
    pub fn get_bwd_neighbors(&self, dense_id: usize) -> &Vec<usize> {
        &self.bwd_edge_list[dense_id]
    }

    // Gets a node by its dense id
    pub fn get_node(&self, dense_id: usize) -> &Node {
        &self.nodes[dense_id]
    }

    // Gets a mutable node by its dense id
    pub fn get_node_mut(&mut self, dense_id: usize) -> &mut Node {
        &mut self.nodes[dense_id]
    }

    // Gets the metadata of an edge.
    pub fn get_edge_metadata(&self, edge: &Edge) -> &EdgeMetadata {
        &self.edge_metadata[edge.metadata_index]
    }

    pub fn get_edge(&self, edge_id: usize) -> &Edge {
        &self.edges[edge_id]
    }

    pub fn get_edge_mut(&mut self, edge_id: usize) -> &mut Edge {
        &mut self.edges[edge_id]
    }

    pub fn add_edge(&mut self, src_id: usize, dest_id: usize, metadata_index: usize) -> usize {
        let edge_id = self.edges.len();
        self.edges.push(Edge::new(src_id, dest_id, metadata_index));

        self.fwd_edge_list[src_id].push(edge_id);
        self.bwd_edge_list[dest_id].push(edge_id);

        edge_id
    }

    pub fn edge_exists(&self, v: usize, w: usize) -> bool {
        self.edges.iter().any(|e| e.src_id == v && e.dest_id == w)
    }

    pub fn add_shortcut_edge(
        &mut self,
        src_id: usize,
        dest_id: usize,
        metadata_index: usize,
        prev_edge: usize,
        next_edge: usize,
    ) -> usize {
        let edge_id = self.edges.len();
        self.edges.push(Edge::new_shortcut(
            src_id,
            dest_id,
            metadata_index,
            prev_edge,
            next_edge,
        ));

        self.fwd_edge_list[src_id].push(edge_id);
        self.bwd_edge_list[dest_id].push(edge_id);

        edge_id
    }

    fn get_nodes_bytes(&self) -> usize {
        self.nodes.len() * std::mem::size_of::<Node>()
    }

    fn get_edges_bytes(&self) -> usize {
        self.edges.len() * std::mem::size_of::<Edge>()
    }

    fn get_edges_metadata_bytes(&self) -> usize {
        self.edge_metadata.len() * std::mem::size_of::<EdgeMetadata>()
    }

    fn get_fwd_bytes(&self) -> usize {
        self.fwd_edge_list
            .iter()
            .map(|v| std::mem::size_of_val(v) + v.len() * std::mem::size_of::<usize>())
            .sum()
    }

    fn get_bwd_bytes(&self) -> usize {
        self.bwd_edge_list
            .iter()
            .map(|v| std::mem::size_of_val(v) + v.len() * std::mem::size_of::<usize>())
            .sum()
    }

    pub fn get_mem_usage_str(&self) -> String {
        let node_bytes = self.get_nodes_bytes();
        let fwd_bytes = self.get_fwd_bytes();
        let bwd_bytes = self.get_bwd_bytes();
        let edge_bytes = self.get_edges_bytes();
        let edge_metadata_bytes = self.get_edges_metadata_bytes();

        let total = node_bytes + fwd_bytes + bwd_bytes + edge_bytes + edge_metadata_bytes;

        format!(
            "nodes={}, fwd_edges={}, bwd_edges={}, edges={}, edge_metadata={}, total={}",
            format_size(node_bytes, DECIMAL),
            format_size(fwd_bytes, DECIMAL),
            format_size(bwd_bytes, DECIMAL),
            format_size(edge_bytes, DECIMAL),
            format_size(edge_metadata_bytes, DECIMAL),
            format_size(total, DECIMAL),
        )
    }
}

impl Node {
    pub fn new(dense_id: usize, osm_id: i64) -> Self {
        Self {
            dense_id,
            osm_id,
            rank: 0,
            is_contracted: false,
            lat: 0.0,
            lon: 0.0,
            is_traffic_light: false,
        }
    }

    pub fn set_rank(&mut self, rank: i32) {
        self.rank = rank;
    }

    pub fn get_rank(&self) -> i32 {
        self.rank
    }

    pub fn set_is_contracted(&mut self, is_contracted: bool) {
        self.is_contracted = is_contracted;
    }

    pub fn get_is_contracted(&self) -> bool {
        self.is_contracted
    }

    pub fn set_lat_lon(&mut self, lat: f64, lon: f64) {
        self.lat = lat;
        self.lon = lon;
    }

    pub fn get_lat_lon(&self) -> (f64, f64) {
        (self.lat, self.lon)
    }

    pub fn set_is_traffic_light(&mut self, is_traffic_light: bool) {
        self.is_traffic_light = is_traffic_light;
    }

    pub fn get_is_traffic_light(&self) -> bool {
        self.is_traffic_light
    }
}

impl Edge {
    pub fn new(src_id: usize, dest_id: usize, metadata_index: usize) -> Self {
        Self {
            src_id,
            dest_id,
            metadata_index,
            is_shortcut: false,
            prev_edge: None,
            next_edge: None,
        }
    }

    pub fn new_shortcut(
        src_id: usize,
        dest_id: usize,
        metadata_index: usize,
        prev_edge: usize,
        next_edge: usize,
    ) -> Self {
        Self {
            src_id,
            dest_id,
            metadata_index,
            is_shortcut: true,
            prev_edge: Some(prev_edge),
            next_edge: Some(next_edge),
        }
    }

    pub fn get_src_id(&self) -> usize {
        self.src_id
    }

    pub fn get_dest_id(&self) -> usize {
        self.dest_id
    }

    pub fn get_metadata_index(&self) -> usize {
        self.metadata_index
    }

    pub fn is_shortcut(&self) -> bool {
        self.is_shortcut
    }

    pub fn get_prev_edge(&self) -> Option<usize> {
        assert!(self.is_shortcut);
        self.prev_edge
    }

    pub fn get_next_edge(&self) -> Option<usize> {
        assert!(self.is_shortcut);
        self.next_edge
    }
}
