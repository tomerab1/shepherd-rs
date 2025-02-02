use itertools::Itertools;
use osmpbf::{Element, ElementReader};
use std::collections::BTreeMap;

use super::utils;

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

#[derive(Debug, Clone)]
struct NodeParseData {
    dense_index: usize,
    lat: f64,
    lon: f64,
    is_traffic_signal: bool,
}

#[derive(Debug, Clone)]
struct WayParseData {
    name: Option<String>,
    max_speed: Option<u8>,
    is_roundabout: bool,
    is_oneway: bool,
    refs: Vec<i64>,
}

#[derive(Debug, Clone)]
struct PBFParseResult {
    osm_id_to_node: BTreeMap<i64, NodeParseData>,
    ways: BTreeMap<i64, WayParseData>,
}

pub fn from_osmpbf(path: &str) -> anyhow::Result<Graph> {
    let parse_result = parse_osmpbf(path)?;

    let nodes = build_nodes(&parse_result.osm_id_to_node);
    let (fwd_edge_list, edge_metadata) = build_edge_list(parse_result, &nodes);
    let bwd_edge_list = build_bwd_edge_list(&fwd_edge_list);

    Ok(Graph {
        fwd_edge_list,
        bwd_edge_list,
        nodes,
        edge_metadata,
    })
}

fn build_bwd_edge_list(forward: &Vec<Vec<Edge>>) -> Vec<Vec<Edge>> {
    let mut reverse = vec![Vec::new(); forward.len()];

    for (u, edges_from_u) in forward.iter().enumerate() {
        for edge in edges_from_u {
            let v = edge.dest_id;

            let rev_edge = Edge {
                src_id: v,
                dest_id: u,
                metadata_index: edge.metadata_index,
            };

            reverse[v].push(rev_edge);
        }
    }

    reverse
}

fn build_edge_list(maps: PBFParseResult, nodes: &[Node]) -> (Vec<Vec<Edge>>, Vec<EdgeMetadata>) {
    let osm_to_dense: BTreeMap<i64, usize> = nodes.iter().map(|n| (n.osm_id, n.dense_id)).collect();
    let mut edge_list: Vec<Vec<Edge>> = vec![Vec::new(); nodes.len()];
    let mut edge_metadata: Vec<EdgeMetadata> = Vec::new();

    for (way_id, way_data) in maps.ways {
        if way_data.refs.is_empty() {
            continue;
        }

        // Get start and end node IDs.
        let start_id = way_data.refs.first().unwrap();
        let end_id = way_data.refs.last().unwrap();

        // Get the dense indexes
        let start_dense_index = osm_to_dense.get(start_id).unwrap_or_else(|| {
            panic!(
                "Node with id={} was not found in Way with id={}",
                start_id, way_id
            )
        });
        let end_dense_index = osm_to_dense.get(end_id).unwrap_or_else(|| {
            panic!(
                "Node with id={} was not found in Way with id={}",
                end_id, way_id
            )
        });

        let polyline_data: Vec<usize> = way_data
            .refs
            .iter()
            .map(|id| {
                *osm_to_dense.get(id).unwrap_or_else(|| {
                    panic!(
                        "Node with id={} was not found in Way with id={}",
                        id, way_id
                    )
                })
            })
            .collect();

        let weight = way_data
            .refs
            .iter()
            .tuple_windows()
            .fold(0f64, |acc, (n1, n2)| {
                // Lookup nodes for calculating weight.
                let node1 = maps.osm_id_to_node.get(n1).unwrap();
                let node2 = maps.osm_id_to_node.get(n2).unwrap();
                acc + utils::haversine_distance(node1.lat, node1.lon, node2.lat, node2.lon)
            });

        let metadata = EdgeMetadata {
            polyline: Polyline {
                ids: polyline_data[1..polyline_data.len() - 1].to_vec(), // Collect only the intermediate nodes.
            },
            weight,
            name: way_data.name.clone(),
            speed_limit: way_data.max_speed,
            is_one_way: way_data.is_oneway,
            is_roundabout: way_data.is_roundabout,
        };

        let mut edge = Edge {
            src_id: *start_dense_index,
            dest_id: *end_dense_index,
            metadata_index: edge_metadata.len(),
        };

        edge_list[*start_dense_index].push(edge.clone());
        if !way_data.is_oneway {
            std::mem::swap(&mut edge.src_id, &mut edge.dest_id);
            edge_list[*end_dense_index].push(edge);
        }

        edge_metadata.push(metadata);
    }

    (edge_list, edge_metadata)
}

fn build_nodes(nodes_map: &BTreeMap<i64, NodeParseData>) -> Vec<Node> {
    let mut nodes: Vec<Node> = nodes_map
        .iter()
        .map(|(&osm_id, data)| Node {
            dense_id: 0,
            osm_id,
            lat: data.lat,
            lon: data.lon,
            is_traffic_light: data.is_traffic_signal,
        })
        .collect();

    sort_by_lat_lon(&mut nodes);

    // assign the dense index
    for (dense_id, node) in nodes.iter_mut().enumerate() {
        node.dense_id = dense_id;
    }

    nodes
}

fn sort_by_lat_lon(nodes: &mut [Node]) {
    // Sort the nodes by lat/lon, may result in better cache locality (?)
    nodes.sort_by(|a, b| {
        a.lat
            .partial_cmp(&b.lat)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(
                a.lon
                    .partial_cmp(&b.lon)
                    .unwrap_or(std::cmp::Ordering::Equal),
            )
    });
}

fn parse_osmpbf(path: &str) -> anyhow::Result<PBFParseResult> {
    let reader = ElementReader::from_path(path)?;

    // Map osm id -> (dense_index, lat, lon, is_traffic_signal)
    let mut osm_id_to_node: BTreeMap<i64, NodeParseData> = BTreeMap::new();
    let mut ways: BTreeMap<i64, WayParseData> = BTreeMap::new();

    reader.for_each(|elem| match elem {
        Element::DenseNode(node) => {
            let is_traffic_signal = node.tags().any(|e| e.1 == "traffic_signals");

            let node_data = NodeParseData {
                dense_index: osm_id_to_node.len(),
                lat: node.lat(),
                lon: node.lon(),
                is_traffic_signal,
            };
            osm_id_to_node.insert(node.id, node_data);
        }
        Element::Way(way) => {
            let name = way.tags().find_map(|(k, v)| {
                if k == "name:en" {
                    Some(v.to_owned())
                } else {
                    None
                }
            });
            let max_speed = way.tags().find_map(|(k, v)| {
                if k == "maxspeed" {
                    v.parse().ok()
                } else {
                    None
                }
            });
            let is_oneway = way.tags().any(|(k, v)| k == "oneway" && v == "yes");
            let is_roundabout = way.tags().any(|(_, v)| v == "roundabout");
            let refs: Vec<i64> = way.refs().collect();

            let way_data = WayParseData {
                name,
                max_speed,
                is_roundabout,
                is_oneway,
                refs,
            };

            ways.insert(way.id(), way_data);
        }
        _ => {}
    })?;

    Ok(PBFParseResult {
        osm_id_to_node,
        ways,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    static TEST_FILE_PATH: &str = "tests/data/nz-car-only.pbf.osm";

    #[test]
    fn test_parse_osmpbf() {
        let target_node_id = 1439390172;
        let target_way_id = 1232194195;

        let maps = parse_osmpbf(TEST_FILE_PATH).unwrap();

        let node = maps
            .osm_id_to_node
            .get(&target_node_id)
            .expect("Failed to find node with id={target_node_id}");

        assert!(node.is_traffic_signal);

        let way = maps
            .ways
            .get(&target_way_id)
            .expect("Failed to find way with id={target_way_id}");

        let expected_nodes = [733900601, 11429137338];
        let expected_name = "Eliyahu Meron";

        assert!(way.name.is_some());
        assert_eq!(way.name.as_ref().unwrap(), expected_name);

        assert_eq!(way.refs, expected_nodes);
    }

    #[test]
    fn test_build_nodes() {
        let mut nodes_map: BTreeMap<i64, NodeParseData> = BTreeMap::new();
        nodes_map.insert(
            100,
            NodeParseData {
                dense_index: 0,
                lat: 10.0,
                lon: 20.0,
                is_traffic_signal: false,
            },
        );
        nodes_map.insert(
            200,
            NodeParseData {
                dense_index: 1,
                lat: 30.0,
                lon: 40.0,
                is_traffic_signal: true,
            },
        );

        let nodes = build_nodes(&nodes_map);

        assert_eq!(nodes.len(), 2);

        assert_eq!(nodes[0].dense_id, 0);
        assert_eq!(nodes[0].osm_id, 100);
        assert_eq!(nodes[0].lat, 10.0);
        assert_eq!(nodes[0].lon, 20.0);
        assert!(!nodes[0].is_traffic_light);

        assert_eq!(nodes[1].dense_id, 1);
        assert_eq!(nodes[1].osm_id, 200);
        assert_eq!(nodes[1].lat, 30.0);
        assert_eq!(nodes[1].lon, 40.0);
        assert!(nodes[1].is_traffic_light);
    }

    #[test]
    fn test_sort_by_lat_lon() {
        let mut nodes_map: BTreeMap<i64, NodeParseData> = BTreeMap::new();
        let nodes_data = [
            (10.0, 20.0, false),
            (40.0, 50.0, false),
            (30.0, 40.0, false),
            (20.0, 30.0, false),
        ];

        let expected_node_data = [(10.0, 20.0), (20.0, 30.0), (30.0, 40.0), (40.0, 50.0)];

        for (i, (lat, lon, is_traffic_signal)) in nodes_data.into_iter().enumerate() {
            nodes_map.insert(
                i as i64,
                NodeParseData {
                    dense_index: i,
                    lat,
                    lon,
                    is_traffic_signal,
                },
            );
        }

        let mut nodes = build_nodes(&nodes_map);

        sort_by_lat_lon(&mut nodes);

        for (i, n) in nodes.iter().enumerate() {
            assert_eq!(n.lat, expected_node_data[i].0);
            assert_eq!(n.lon, expected_node_data[i].1);
        }
    }

    #[test]
    fn test_build_edge_list() {
        let mut nodes_map: BTreeMap<i64, NodeParseData> = BTreeMap::new();
        let nodes_data = [
            (10.0, 20.0, false),
            (20.0, 30.0, false),
            (30.0, 40.0, false),
            (40.0, 50.0, false),
        ];

        for (i, (lat, lon, is_traffic_signal)) in nodes_data.into_iter().enumerate() {
            nodes_map.insert(
                i as i64,
                NodeParseData {
                    dense_index: i,
                    lat,
                    lon,
                    is_traffic_signal,
                },
            );
        }

        let mut ways: BTreeMap<i64, WayParseData> = BTreeMap::new();

        ways.insert(
            0,
            WayParseData {
                name: None,
                max_speed: None,
                is_roundabout: false,
                is_oneway: false,
                refs: vec![0, 1, 2, 3],
            },
        );

        let maps = PBFParseResult {
            osm_id_to_node: nodes_map.clone(),
            ways,
        };

        let nodes = build_nodes(&nodes_map);
        let (edge_list, edge_metadata) = build_edge_list(maps, &nodes);

        assert!(edge_list[0].len() == 1 && edge_list[3].len() == 1);
        assert_eq!(
            edge_list[0][0],
            Edge {
                src_id: 0,
                dest_id: 3,
                metadata_index: 0
            }
        );

        assert_eq!(
            edge_list[3][0],
            Edge {
                src_id: 3,
                dest_id: 0,
                metadata_index: 0
            }
        );

        assert_eq!(edge_metadata[0].polyline.ids, &[1, 2]);
    }

    #[test]
    fn test_build_bwd_edge_list() {
        let mut nodes_map: BTreeMap<i64, NodeParseData> = BTreeMap::new();
        let nodes_data = [
            (10.0, 20.0, false),
            (20.0, 30.0, false),
            (30.0, 40.0, false),
            (40.0, 50.0, false),
        ];

        for (i, (lat, lon, is_traffic_signal)) in nodes_data.into_iter().enumerate() {
            nodes_map.insert(
                i as i64,
                NodeParseData {
                    dense_index: i,
                    lat,
                    lon,
                    is_traffic_signal,
                },
            );
        }

        let mut ways: BTreeMap<i64, WayParseData> = BTreeMap::new();

        ways.insert(
            0,
            WayParseData {
                name: None,
                max_speed: None,
                is_roundabout: false,
                is_oneway: true,
                refs: vec![0, 1, 2, 3],
            },
        );

        let maps = PBFParseResult {
            osm_id_to_node: nodes_map.clone(),
            ways,
        };

        let nodes = build_nodes(&nodes_map);
        let (edge_list, _) = build_edge_list(maps, &nodes);

        let bwd_edge_list = build_bwd_edge_list(&edge_list);

        assert_eq!(bwd_edge_list[3][0].src_id, 3);
        assert_eq!(bwd_edge_list[3][0].dest_id, 0);
    }
}
