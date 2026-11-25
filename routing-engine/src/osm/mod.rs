//! OSM import: reads a `.osm.pbf` into the minimal in-memory form the graph builder needs.

pub mod profile;

use crate::geo::Coord;
use anyhow::Context;
use osmpbf::{Element, ElementReader, RelMemberType};
use profile::{Profile, WayProfile};
use std::collections::HashSet;

#[derive(Clone, Debug)]
pub struct OsmWay {
    pub id: i64,
    pub refs: Vec<i64>,
    pub profile: WayProfile,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RestrictionKind {
    /// `no_*`: the turn from -> to is forbidden.
    No,
    /// `only_*`: from `from`, the only allowed turn at `via` is onto `to`.
    Only,
}

/// What a restriction goes through: a single node, or one or more ways (in order).
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Via {
    Node(i64),
    Ways(Vec<i64>),
}

#[derive(Clone, Debug)]
pub struct Restriction {
    pub from_way: i64,
    pub via: Via,
    pub to_way: i64,
    pub kind: RestrictionKind,
}

impl Restriction {
    pub fn via_node(&self) -> Option<i64> {
        match self.via {
            Via::Node(n) => Some(n),
            Via::Ways(_) => None,
        }
    }
}

/// Everything the builder needs from OSM. Can be built from a PBF or by hand in tests.
#[derive(Default, Debug)]
pub struct OsmData {
    pub ways: Vec<OsmWay>,
    /// Coordinates of every node referenced by a routable way, sorted by OSM id.
    pub nodes: Vec<(i64, Coord)>,
    /// Traffic lights: (OSM node id, `SIG_*` flags), sorted by id.
    pub signals: Vec<(i64, u8)>,
    pub restrictions: Vec<Restriction>,
    /// Restrictions we could not use (missing or malformed members); reported in stats.
    pub skipped_restrictions: usize,
}

impl OsmData {
    pub fn node(&self, id: i64) -> Option<Coord> {
        self.nodes
            .binary_search_by_key(&id, |n| n.0)
            .ok()
            .map(|i| self.nodes[i].1)
    }
    /// `SIG_*` flags of a node, 0 if it has no traffic light.
    pub fn signal_flags(&self, id: i64) -> u8 {
        self.signals
            .binary_search_by_key(&id, |s| s.0)
            .map(|i| self.signals[i].1)
            .unwrap_or(0)
    }
}

fn parse_restriction(rel: &osmpbf::Relation) -> Result<Option<Restriction>, ()> {
    let mut is_restriction = false;
    let mut value: Option<&str> = None;
    let mut motorcar_value: Option<&str> = None;
    let mut excepted = false;
    for (k, v) in rel.tags() {
        match k {
            "type" => is_restriction = v == "restriction" || v == "restriction:motorcar",
            "restriction" => value = Some(v),
            "restriction:motorcar" => motorcar_value = Some(v),
            "except" => excepted = v.split(';').any(|e| matches!(e.trim(), "motorcar" | "motor_vehicle")),
            _ => {}
        }
    }
    if !is_restriction || excepted {
        return Ok(None);
    }
    let Some(value) = motorcar_value.or(value) else {
        return Ok(None);
    };
    let kind = if value.starts_with("no_") {
        RestrictionKind::No
    } else if value.starts_with("only_") {
        RestrictionKind::Only
    } else {
        return Ok(None);
    };

    let (mut from, mut via_node, mut to) = (None, None, None);
    let mut via_ways = Vec::new();
    for m in rel.members() {
        let role = m.role().unwrap_or("");
        match (role, m.member_type) {
            ("from", RelMemberType::Way) => from = Some(m.member_id),
            ("to", RelMemberType::Way) => to = Some(m.member_id),
            ("via", RelMemberType::Node) => via_node = Some(m.member_id),
            ("via", RelMemberType::Way) => via_ways.push(m.member_id),
            _ => {}
        }
    }
    let via = match (via_node, via_ways.is_empty()) {
        (Some(n), true) => Via::Node(n),
        (None, false) => Via::Ways(via_ways),
        _ => return Err(()),
    };
    match (from, to) {
        (Some(from_way), Some(to_way)) => Ok(Some(Restriction {
            from_way,
            via,
            to_way,
            kind,
        })),
        _ => Err(()),
    }
}

/// Node is a vehicle traffic light (`highway=traffic_signals`).
pub const SIG_VEHICLE: u8 = 1;
/// Node is a pedestrian crossing with lights (`crossing=traffic_signals`).
pub const SIG_CROSSING: u8 = 2;
/// The light only applies in the way's forward / backward direction.
pub const SIG_FORWARD: u8 = 4;
pub const SIG_BACKWARD: u8 = 8;

fn signal_flags<'a>(tags: impl Iterator<Item = (&'a str, &'a str)>) -> u8 {
    let mut f = 0u8;
    let mut dir = 0u8;
    for (k, v) in tags {
        match (k, v) {
            ("highway", "traffic_signals") => f |= SIG_VEHICLE,
            ("crossing", v) if v.split(';').any(|x| x == "traffic_signals") => f |= SIG_CROSSING,
            ("traffic_signals:direction" | "direction", "forward") => dir = SIG_FORWARD,
            ("traffic_signals:direction" | "direction", "backward") => dir = SIG_BACKWARD,
            _ => {}
        }
    }
    if f & SIG_VEHICLE != 0 {
        f |= dir;
    }
    f
}

#[derive(Default)]
struct Pass1 {
    ways: Vec<OsmWay>,
    restrictions: Vec<Restriction>,
    skipped: usize,
}

/// Reads a PBF in two parallel passes: ways + relations, then only the nodes those ways use.
pub fn read_pbf(path: &str, profile: Profile) -> anyhow::Result<OsmData> {
    let pass1 = ElementReader::from_path(path)
        .with_context(|| format!("opening {path}"))?
        .par_map_reduce(
            |el| {
                let mut out = Pass1::default();
                match el {
                    Element::Way(w) => {
                        if let Some(way_profile) = profile.way(w.tags()) {
                            let refs: Vec<i64> = w.refs().collect();
                            if refs.len() >= 2 {
                                out.ways.push(OsmWay {
                                    id: w.id(),
                                    refs,
                                    profile: way_profile,
                                });
                            }
                        }
                    }
                    Element::Relation(r) if profile.uses_restrictions() => match parse_restriction(&r) {
                        Ok(Some(res)) => out.restrictions.push(res),
                        Ok(None) => {}
                        Err(()) => out.skipped += 1,
                    },
                    _ => {}
                }
                out
            },
            Pass1::default,
            |mut a, mut b| {
                if a.ways.len() < b.ways.len() {
                    std::mem::swap(&mut a, &mut b);
                }
                a.ways.append(&mut b.ways);
                a.restrictions.append(&mut b.restrictions);
                a.skipped += b.skipped;
                a
            },
        )?;

    let mut ways = pass1.ways;
    ways.sort_unstable_by_key(|w| w.id);

    let mut needed: Vec<i64> = ways.iter().flat_map(|w| w.refs.iter().copied()).collect();
    needed.sort_unstable();
    needed.dedup();
    let needed_set: HashSet<i64> = needed.iter().copied().collect();
    drop(needed);

    type NodeOut = (Vec<(i64, Coord)>, Vec<(i64, u8)>);
    let (mut nodes, mut signals) = ElementReader::from_path(path)?.par_map_reduce(
        |el| -> NodeOut {
            let (id, lat, lon, flags) = match el {
                Element::DenseNode(n) => (n.id(), n.lat(), n.lon(), signal_flags(n.tags())),
                Element::Node(n) => (n.id(), n.lat(), n.lon(), signal_flags(n.tags())),
                _ => return (Vec::new(), Vec::new()),
            };
            if !needed_set.contains(&id) {
                return (Vec::new(), Vec::new());
            }
            let sig = if flags != 0 { vec![(id, flags)] } else { Vec::new() };
            (vec![(id, Coord::from_deg(lat, lon))], sig)
        },
        || (Vec::new(), Vec::new()),
        |mut a, mut b| {
            if a.0.len() < b.0.len() {
                std::mem::swap(&mut a, &mut b);
            }
            a.0.append(&mut b.0);
            a.1.append(&mut b.1);
            a
        },
    )?;
    nodes.sort_unstable_by_key(|n| n.0);
    signals.sort_unstable_by_key(|s| s.0);

    Ok(OsmData {
        ways,
        nodes,
        signals,
        restrictions: pass1.restrictions,
        skipped_restrictions: pass1.skipped,
    })
}
