use itertools::Itertools;
use multimap::MultiMap;
use osmpbf::{Element, ElementReader, Way};
use std::collections::{BTreeMap, HashMap};

use super::graph::{Edge, EdgeMetadata, Graph, Node};
use crate::engine::utils;

#[derive(Debug, Clone)]
struct NodeParseData {
    dense_index: usize,
    lat: f32,
    lon: f32,
    is_traffic_signal: bool,
}

#[derive(Debug, Clone)]
struct WayParseData {
    id: i64,
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

struct BuildEdgeListResult {
    fwd_edge_list: Vec<Vec<usize>>,
    bwd_edge_list: Vec<Vec<usize>>,
    edges: Vec<Edge>,
    edge_metadata: Vec<EdgeMetadata>,
}

pub fn from_osmpbf(path: &str) -> anyhow::Result<Graph> {
    let parse_result = parse_osmpbf(path)?;
    let intersections_map = create_intersections_map(path)?;

    let nodes = build_nodes(&parse_result.osm_id_to_node);
    let build_edge_lists_result = build_edge_lists(parse_result, &nodes, intersections_map);

    Ok(Graph {
        fwd_edge_list: build_edge_lists_result.fwd_edge_list,
        bwd_edge_list: build_edge_lists_result.bwd_edge_list,
        edges: build_edge_lists_result.edges,
        edge_metadata: build_edge_lists_result.edge_metadata,
        nodes,
    })
}

fn parse_polyline_data(way_data: &WayParseData) -> Vec<i64> {
    way_data.refs.to_vec()
}

fn calc_weight_with_turn(prev_id: i64, curr_id: i64, next_id: i64, maps: &PBFParseResult) -> f32 {
    let prev = maps.osm_id_to_node.get(&prev_id).unwrap();
    let curr = maps.osm_id_to_node.get(&curr_id).unwrap();
    let next: &NodeParseData = maps.osm_id_to_node.get(&next_id).unwrap();

    let dist = utils::haversine_distance(curr.lat, curr.lon, next.lat, next.lon);
    let turn_cost =
        utils::calc_turn_cost(prev.lat, prev.lon, curr.lat, curr.lon, next.lat, next.lon);

    turn_cost * dist
}

fn calc_weight_without_turn(curr_id: i64, next_id: i64, maps: &PBFParseResult) -> f32 {
    let curr = maps.osm_id_to_node.get(&curr_id).unwrap();
    let next: &NodeParseData = maps.osm_id_to_node.get(&next_id).unwrap();

    utils::haversine_distance(curr.lat, curr.lon, next.lat, next.lon)
}

fn build_edge_lists(
    maps: PBFParseResult,
    nodes: &[Node],
    intersections_map: MultiMap<i64, i64>,
) -> BuildEdgeListResult {
    let osm_to_dense: BTreeMap<i64, usize> = nodes.iter().map(|n| (n.osm_id, n.dense_id)).collect();
    let mut fwd_edge_list: Vec<Vec<usize>> = vec![Vec::new(); nodes.len()];
    let mut bwd_edge_list: Vec<Vec<usize>> = vec![Vec::new(); nodes.len()];
    let mut edge_metadata: Vec<EdgeMetadata> = Vec::new();
    let mut edges: Vec<Edge> = Vec::new();

    for way_data in maps.ways.values() {
        if way_data.refs.is_empty() {
            continue;
        }

        if way_data.is_roundabout {
            let polyline_data = parse_polyline_data(way_data);
            for (curr_id, next_id) in polyline_data.iter().tuple_windows() {
                let weight = calc_weight_without_turn(*curr_id, *next_id, &maps);
                let curr_node = osm_to_dense.get(curr_id).unwrap();
                let next_node = osm_to_dense.get(next_id).unwrap();

                let metadata_index = edge_metadata.len();
                let metadata = EdgeMetadata {
                    weight,
                    is_one_way: way_data.is_oneway,
                    is_roundabout: way_data.is_roundabout,
                    name: way_data.name.clone(),
                    speed_limit: way_data.max_speed,
                    prev_edge: None,
                    next_edge: None,
                };
                edge_metadata.push(metadata);

                let edge_index_fwd = edges.len();
                edges.push(Edge::new(*curr_node, *next_node, metadata_index));
                fwd_edge_list[*curr_node].push(edge_index_fwd);
                bwd_edge_list[*next_node].push(edge_index_fwd);

                if !(way_data.is_oneway || way_data.is_roundabout) && curr_id != next_id {
                    let edge_index_bwd = edges.len();
                    edges.push(Edge::new(*next_node, *curr_node, metadata_index));
                    fwd_edge_list[*next_node].push(edge_index_bwd);
                    bwd_edge_list[*curr_node].push(edge_index_bwd);
                }
            }
        } else {
            let osm_ids = intersections_map.get_vec(&way_data.id);
            if let Some(osm_ids) = osm_ids {
                for (prev_id, curr_id, next_id) in osm_ids.iter().tuple_windows() {
                    let weight = calc_weight_with_turn(*prev_id, *curr_id, *next_id, &maps);
                    let prev_node = osm_to_dense.get(prev_id).unwrap();
                    let next_node = osm_to_dense.get(next_id).unwrap();

                    let metadata_index = edge_metadata.len();
                    let metadata = EdgeMetadata {
                        weight,
                        is_one_way: way_data.is_oneway,
                        is_roundabout: way_data.is_roundabout,
                        name: way_data.name.clone(),
                        speed_limit: way_data.max_speed,
                        prev_edge: None,
                        next_edge: None,
                    };
                    edge_metadata.push(metadata);

                    let edge_index_fwd = edges.len();
                    edges.push(Edge::new(*prev_node, *next_node, metadata_index));
                    fwd_edge_list[*prev_node].push(edge_index_fwd);
                    bwd_edge_list[*next_node].push(edge_index_fwd);

                    if !way_data.is_oneway {
                        let edge_index_bwd = edges.len();
                        edges.push(Edge::new(*next_node, *prev_node, metadata_index));
                        fwd_edge_list[*next_node].push(edge_index_bwd);
                        bwd_edge_list[*prev_node].push(edge_index_bwd);
                    }
                }
            } else {
                let polyline_data = parse_polyline_data(way_data);
                let curr_id = polyline_data.first();
                let next_id = polyline_data.last();

                if let (Some(curr_id), Some(next_id)) = (curr_id, next_id) {
                    let weight = calc_weight_without_turn(*curr_id, *next_id, &maps);
                    let curr_node = osm_to_dense.get(curr_id).unwrap();
                    let next_node = osm_to_dense.get(next_id).unwrap();

                    let metadata_index = edge_metadata.len();
                    let metadata = EdgeMetadata {
                        weight,
                        is_one_way: way_data.is_oneway,
                        is_roundabout: way_data.is_roundabout,
                        name: way_data.name.clone(),
                        speed_limit: way_data.max_speed,
                        prev_edge: None,
                        next_edge: None,
                    };
                    edge_metadata.push(metadata);

                    let edge_index_fwd = edges.len();
                    edges.push(Edge::new(*curr_node, *next_node, metadata_index));
                    fwd_edge_list[*curr_node].push(edge_index_fwd);
                    bwd_edge_list[*next_node].push(edge_index_fwd);

                    if !(way_data.is_oneway || way_data.is_roundabout) && curr_id != next_id {
                        let edge_index_bwd = edges.len();
                        edges.push(Edge::new(*next_node, *curr_node, metadata_index));
                        fwd_edge_list[*next_node].push(edge_index_bwd);
                        bwd_edge_list[*curr_node].push(edge_index_bwd);
                    }
                }
            }
        }
    }

    BuildEdgeListResult {
        fwd_edge_list,
        bwd_edge_list,
        edges,
        edge_metadata,
    }
}

fn build_nodes(nodes_map: &BTreeMap<i64, NodeParseData>) -> Vec<Node> {
    nodes_map
        .iter()
        .enumerate()
        .map(|(i, (&osm_id, data))| Node {
            dense_id: i,
            osm_id,
            rank: 0,
            is_contracted: false,
            lat: data.lat,
            lon: data.lon,
            is_traffic_light: data.is_traffic_signal,
        })
        .collect()
}

fn parse_way_name(way: &Way) -> Option<String> {
    way.tags().find_map(|(k, v)| {
        if k == "name:en" {
            Some(v.to_owned())
        } else {
            None
        }
    })
}

fn parse_way_max_speed(way: &Way) -> Option<u8> {
    way.tags().find_map(|(k, v)| {
        if k == "maxspeed" {
            v.parse().ok()
        } else {
            None
        }
    })
}

fn create_intersections_map(path: &str) -> anyhow::Result<MultiMap<i64, i64>> {
    let reader = ElementReader::from_path(path)?;

    // Key = way_id, value = osm_id, in a multimap several values could be associated with a key
    let mut intersections_map: MultiMap<i64, i64> = MultiMap::new();
    let mut node_count = HashMap::new();

    _ = reader.for_each(|elem| {
        if let Element::Way(way) = elem {
            let refs: Vec<i64> = way.refs().collect();
            intersections_map.insert_many_from_slice(way.id(), &refs);
            refs.iter().for_each(|id| {
                *node_count.entry(*id).or_insert(0) += 1;
            });
        }
    });

    let mut filtered_way_nodes = MultiMap::new();
    for (way_id, nodes) in intersections_map {
        for node in &nodes {
            if node_count[node] > 1 {
                filtered_way_nodes.insert(way_id, *node);
            }
        }
    }

    Ok(filtered_way_nodes)
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
                lat: node.lat() as f32,
                lon: node.lon() as f32,
                is_traffic_signal,
            };
            osm_id_to_node.insert(node.id(), node_data);
        }
        Element::Node(node) => {
            let is_traffic_signal = node.tags().any(|e| e.1 == "traffic_signals");
            let node_data = NodeParseData {
                dense_index: osm_id_to_node.len(),
                lat: node.lat() as f32,
                lon: node.lon() as f32,
                is_traffic_signal,
            };
            osm_id_to_node.insert(node.id(), node_data);
        }
        Element::Way(way) => {
            let name = parse_way_name(&way);
            let max_speed = parse_way_max_speed(&way);
            let is_oneway = way.tags().any(|(k, v)| k == "oneway" && v == "yes");
            let is_roundabout = way.tags().any(|(_, v)| v == "roundabout");
            let refs: Vec<i64> = way.refs().collect();

            let way_data = WayParseData {
                id: way.id(),
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
    fn test_build_edge_lists() {
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
        let result = build_edge_lists(maps, &nodes);

        let fwd_edge_list = result.fwd_edge_list;
        let bwd_edge_list = result.bwd_edge_list;

        let edges = result.edges;
        assert_eq!(fwd_edge_list[0].len(), 1);
        assert_eq!(bwd_edge_list[3].len(), 1);

        let edge_id_fwd = fwd_edge_list[0][0];
        let edge_fwd = &edges[edge_id_fwd];

        assert_eq!(edge_fwd.src_id, 0);
        assert_eq!(edge_fwd.dest_id, 3);
        assert_eq!(edge_fwd.metadata_index, 0);

        let edge_id_bwd = bwd_edge_list[3][0];
        let edge_bwd = &edges[edge_id_bwd];

        assert_eq!(edge_bwd.src_id, 3);
        assert_eq!(edge_bwd.dest_id, 0);
        assert_eq!(edge_bwd.metadata_index, 0);
    }
}
