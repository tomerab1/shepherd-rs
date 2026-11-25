//! Turning leg paths into an OSRM-compatible route response with turn-by-turn steps.

use crate::ebg::dseg_bearing;
use crate::engine::{Engine, LegPath, MetricState, Piece, RouteError};
use crate::geo::{encode_polyline, fast_dist, lerp, turn_angle, Coord};
use crate::osm::profile::RoadClass;
use serde::Serialize;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GeometryFormat {
    Polyline,
    Polyline6,
    GeoJson,
}

#[derive(Clone, Debug)]
pub struct RouteOptions {
    pub steps: bool,
    pub overview: bool,
    pub geometry: GeometryFormat,
    /// Name of an area to avoid (see `--avoid-area`), e.g. "judea-samaria".
    pub exclude: Option<String>,
}

impl Default for RouteOptions {
    fn default() -> Self {
        Self {
            steps: true,
            overview: true,
            geometry: GeometryFormat::Polyline,
            exclude: None,
        }
    }
}

#[derive(Serialize, Debug, Clone)]
#[serde(untagged)]
pub enum Geometry {
    Encoded(String),
    GeoJson {
        #[serde(rename = "type")]
        kind: &'static str,
        coordinates: Vec<[f64; 2]>,
    },
}

fn make_geometry(coords: &[Coord], f: GeometryFormat) -> Geometry {
    match f {
        GeometryFormat::Polyline => Geometry::Encoded(encode_polyline(coords, 5)),
        GeometryFormat::Polyline6 => Geometry::Encoded(encode_polyline(coords, 6)),
        GeometryFormat::GeoJson => Geometry::GeoJson {
            kind: "LineString",
            coordinates: coords.iter().map(|c| [c.lon(), c.lat()]).collect(),
        },
    }
}

#[derive(Serialize, Debug, Clone)]
pub struct Maneuver {
    pub location: [f64; 2],
    pub bearing_before: u16,
    pub bearing_after: u16,
    #[serde(rename = "type")]
    pub kind: &'static str,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub modifier: Option<&'static str>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub exit: Option<u32>,
    /// Human-readable instruction (English).
    pub instruction: String,
}

#[derive(Serialize, Debug, Clone)]
pub struct Step {
    pub distance: f64,
    pub duration: f64,
    pub name: String,
    pub mode: &'static str,
    pub maneuver: Maneuver,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub geometry: Option<Geometry>,
}

#[derive(Serialize, Debug, Clone)]
pub struct Leg {
    pub distance: f64,
    pub duration: f64,
    pub weight: f64,
    pub summary: String,
    pub steps: Vec<Step>,
}

#[derive(Serialize, Debug, Clone)]
pub struct Route {
    pub distance: f64,
    pub duration: f64,
    pub weight: f64,
    pub weight_name: &'static str,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub geometry: Option<Geometry>,
    pub legs: Vec<Leg>,
    /// Traffic lights on the route as `[lon, lat]`, in driving order.
    pub traffic_lights: Vec<[f64; 2]>,
}

#[derive(Serialize, Debug, Clone)]
pub struct Waypoint {
    pub location: [f64; 2],
    pub name: String,
    pub distance: f64,
}

#[derive(Serialize, Debug, Clone)]
pub struct RouteResponse {
    pub code: &'static str,
    pub routes: Vec<Route>,
    pub waypoints: Vec<Waypoint>,
    /// Version of the metric (traffic state) the route was computed with.
    pub metric_version: u64,
}

/// A piece with its resolved geometry and cost.
struct Resolved {
    coords: Vec<Coord>,
    distance: f64,
    duration: f64,
    name: u32,
    class: RoadClass,
    roundabout: bool,
    /// Bearings at the start and end of the full directed segment.
    bearing_in: f64,
    bearing_out: f64,
    /// Number of legal turns out of this directed segment (choice at its end junction).
    out_degree: usize,
}

/// Directed geometry of `seg` between fractions `a..b` of its length (in driving direction).
fn partial(geom: &[Coord], forward: bool, a: f64, b: f64, out: &mut Vec<Coord>) -> f64 {
    let n = geom.len();
    let pt = |i: usize| if forward { geom[i] } else { geom[n - 1 - i] };
    let mut cum = Vec::with_capacity(n);
    let mut total = 0.0;
    cum.push(0.0);
    for i in 1..n {
        total += fast_dist(pt(i - 1), pt(i));
        cum.push(total);
    }
    let (da, db) = (a * total, b * total);
    let at = |d: f64| -> Coord {
        let i = cum.partition_point(|&c| c < d).clamp(1, n - 1);
        let span = cum[i] - cum[i - 1];
        let t = if span > 0.0 { (d - cum[i - 1]) / span } else { 0.0 };
        lerp(pt(i - 1), pt(i), t.clamp(0.0, 1.0))
    };
    out.push(at(da));
    for i in 1..n - 1 {
        if cum[i] > da && cum[i] < db {
            out.push(pt(i));
        }
    }
    out.push(at(db));
    (db - da).max(0.0)
}

fn resolve(engine: &Engine, m: &MetricState, p: &Piece) -> Resolved {
    let g = &engine.graph;
    let seg = &g.segments[p.segment as usize];
    let geom = g.seg_geometry(p.segment);
    let mut coords = Vec::with_capacity(geom.len());
    let distance = partial(geom, p.forward, p.from, p.to, &mut coords);
    let dur_ds = m.elem_dur[p.elem as usize] as f64 * (p.to - p.from).max(0.0) + p.turn_ds as f64;
    Resolved {
        coords,
        distance,
        duration: dur_ds / 10.0,
        name: seg.name,
        class: RoadClass::from_u8(seg.class),
        roundabout: seg.is_roundabout(),
        bearing_in: dseg_bearing(geom, p.forward, false),
        bearing_out: dseg_bearing(geom, p.forward, true),
        out_degree: g.elem_choices(p.elem),
    }
}

fn modifier(angle: f64) -> &'static str {
    let a = angle.abs();
    let side_right = angle > 0.0;
    match a {
        a if a < 20.0 => "straight",
        a if a < 50.0 => {
            if side_right {
                "slight right"
            } else {
                "slight left"
            }
        }
        a if a < 130.0 => {
            if side_right {
                "right"
            } else {
                "left"
            }
        }
        a if a < 170.0 => {
            if side_right {
                "sharp right"
            } else {
                "sharp left"
            }
        }
        _ => "uturn",
    }
}

fn cardinal(bearing: f64) -> &'static str {
    const D: [&str; 8] = ["north", "northeast", "east", "southeast", "south", "southwest", "west", "northwest"];
    D[(((bearing + 22.5) % 360.0) / 45.0) as usize % 8]
}

fn ordinal(n: u32) -> String {
    let suffix = match (n % 10, n % 100) {
        (1, x) if x != 11 => "st",
        (2, x) if x != 12 => "nd",
        (3, x) if x != 13 => "rd",
        _ => "th",
    };
    format!("{n}{suffix}")
}

fn onto(name: &str) -> String {
    if name.is_empty() {
        String::new()
    } else {
        format!(" onto {name}")
    }
}

fn is_highway(c: RoadClass) -> bool {
    matches!(c, RoadClass::Motorway | RoadClass::Trunk)
}

struct StepBuilder {
    kind: &'static str,
    modifier: Option<&'static str>,
    exit: Option<u32>,
    location: Coord,
    bearing_before: f64,
    bearing_after: f64,
    name: u32,
    distance: f64,
    duration: f64,
    coords: Vec<Coord>,
}

fn build_steps(engine: &Engine, pieces: &[Resolved], opts: &RouteOptions) -> Vec<Step> {
    let g = &engine.graph;
    let mut steps: Vec<StepBuilder> = Vec::new();
    let first = &pieces[0];
    let start = first.coords[0];
    let dep_bearing = if first.coords.len() > 1 {
        crate::geo::bearing(first.coords[0], first.coords[first.coords.len().min(2) - 1])
    } else {
        first.bearing_in
    };
    steps.push(StepBuilder {
        kind: "depart",
        modifier: None,
        exit: None,
        location: start,
        bearing_before: 0.0,
        bearing_after: dep_bearing,
        name: first.name,
        distance: 0.0,
        duration: 0.0,
        coords: vec![start],
    });

    let mut in_roundabout = false;
    for (i, p) in pieces.iter().enumerate() {
        if i > 0 {
            let prev = &pieces[i - 1];
            let angle = turn_angle(prev.bearing_out, p.bearing_in);
            let junction = p.coords[0];
            let cur = steps.last_mut().unwrap();
            if in_roundabout {
                if p.roundabout {
                    // Passing an exit we don't take?
                    if prev.out_degree > 1 {
                        *cur.exit.as_mut().unwrap() += 1;
                    }
                    cur.name = if cur.name == crate::network::NO_NAME { p.name } else { cur.name };
                } else {
                    // Leaving the roundabout: the step keeps its exit count; name the exit road.
                    cur.name = p.name;
                    in_roundabout = false;
                }
            } else if p.roundabout && !prev.roundabout {
                in_roundabout = true;
                steps.push(StepBuilder {
                    kind: "roundabout",
                    modifier: Some(modifier(angle)),
                    exit: Some(1),
                    location: junction,
                    bearing_before: prev.bearing_out,
                    bearing_after: p.bearing_in,
                    name: crate::network::NO_NAME,
                    distance: 0.0,
                    duration: 0.0,
                    coords: vec![junction],
                });
            } else {
                let name_change = p.name != prev.name;
                let turning = angle.abs() >= 35.0;
                let choice = prev.out_degree > 1;
                let kind = if prev.class == RoadClass::Link && p.class == RoadClass::Link {
                    // Splits inside an interchange: "keep left/right" only where there is a choice.
                    (choice && angle.abs() >= 8.0).then_some("fork")
                } else if !is_highway(prev.class) && p.class == RoadClass::Link && is_highway_ahead(pieces, i) {
                    Some("on ramp")
                } else if is_highway(prev.class) && p.class == RoadClass::Link {
                    Some("off ramp")
                } else if prev.class == RoadClass::Link && is_highway(p.class) {
                    Some("merge")
                } else if (turning && choice) || (angle.abs() >= 100.0) {
                    Some("turn")
                } else if name_change && p.name != crate::network::NO_NAME {
                    Some(if choice { "new name" } else { "continue" })
                } else {
                    None
                };
                if let Some(kind) = kind {
                    let m = if kind == "fork" {
                        if angle < 0.0 { "slight left" } else { "slight right" }
                    } else {
                        modifier(angle)
                    };
                    steps.push(StepBuilder {
                        kind,
                        modifier: Some(m),
                        exit: None,
                        location: junction,
                        bearing_before: prev.bearing_out,
                        bearing_after: p.bearing_in,
                        name: p.name,
                        distance: 0.0,
                        duration: 0.0,
                        coords: vec![junction],
                    });
                }
            }
        }
        let cur = steps.last_mut().unwrap();
        cur.distance += p.distance;
        cur.duration += p.duration;
        cur.coords.extend_from_slice(&p.coords[1..]);
    }
    let end = *pieces.last().unwrap().coords.last().unwrap();
    let last_bearing = pieces.last().unwrap().bearing_out;
    steps.push(StepBuilder {
        kind: "arrive",
        modifier: None,
        exit: None,
        location: end,
        bearing_before: last_bearing,
        bearing_after: 0.0,
        name: pieces.last().unwrap().name,
        distance: 0.0,
        duration: 0.0,
        coords: vec![end, end],
    });

    steps
        .into_iter()
        .map(|s| {
            let name = g.name(s.name).to_string();
            let instruction = match s.kind {
                "depart" => {
                    if name.is_empty() {
                        format!("Head {}", cardinal(s.bearing_after))
                    } else {
                        format!("Head {} on {name}", cardinal(s.bearing_after))
                    }
                }
                "arrive" => "You have arrived at your destination".to_string(),
                "roundabout" => format!(
                    "At the roundabout, take the {} exit{}",
                    ordinal(s.exit.unwrap_or(1)),
                    onto(&name)
                ),
                "on ramp" => format!("Take the ramp{}", onto(&name)),
                "off ramp" => format!("Take the exit{}", onto(&name)),
                "merge" => format!("Merge{}", onto(&name)),
                "continue" | "new name" => format!("Continue{}", onto(&name)),
                "fork" => format!(
                    "Keep {}{}",
                    if s.modifier.is_some_and(|m| m.contains("left")) { "left" } else { "right" },
                    onto(&name)
                ),
                _ => match s.modifier.unwrap_or("straight") {
                    "straight" => format!("Continue straight{}", onto(&name)),
                    "uturn" => format!("Make a U-turn{}", onto(&name)),
                    m @ ("left" | "right") => format!("Turn {m}{}", onto(&name)),
                    m => format!("Make a {m}{}", onto(&name)),
                },
            };
            Step {
                distance: round1(s.distance),
                duration: round1(s.duration),
                name,
                mode: if g.meta.profile == "walking" { "walking" } else { "driving" },
                maneuver: Maneuver {
                    location: [s.location.lon(), s.location.lat()],
                    bearing_before: s.bearing_before.round() as u16 % 360,
                    bearing_after: s.bearing_after.round() as u16 % 360,
                    kind: s.kind,
                    modifier: s.modifier,
                    exit: s.exit,
                    instruction,
                },
                geometry: opts.overview.then(|| make_geometry(&s.coords, opts.geometry)),
            }
        })
        .collect()
}

fn is_highway_ahead(pieces: &[Resolved], from: usize) -> bool {
    pieces[from..]
        .iter()
        .take_while(|p| p.class == RoadClass::Link || is_highway(p.class))
        .any(|p| is_highway(p.class))
}

fn round1(x: f64) -> f64 {
    (x * 10.0).round() / 10.0
}

impl Engine {
    /// Computes a route through all `coords` (at least two).
    pub fn route(&self, coords: &[Coord], opts: &RouteOptions) -> Result<RouteResponse, RouteError> {
        if coords.len() < 2 {
            return Err(RouteError::InvalidQuery("need at least two coordinates".into()));
        }
        let m = match &opts.exclude {
            None => self.metric(),
            Some(area) => self.metric_avoiding(area).ok_or_else(|| {
                RouteError::InvalidQuery(format!(
                    "unknown exclude '{area}' (available: {})",
                    self.area_names().join(", ")
                ))
            })?,
        };
        let mut snaps = Vec::with_capacity(coords.len());
        for (i, &c) in coords.iter().enumerate() {
            snaps.push(self.snap(c).ok_or(RouteError::NoSegment(i))?);
        }
        let mut legs_paths: Vec<LegPath> = Vec::with_capacity(coords.len() - 1);
        for i in 0..snaps.len() - 1 {
            legs_paths.push(self.leg(&m, &snaps[i], &snaps[i + 1]).ok_or(RouteError::NoRoute(i, i + 1))?);
        }

        let mut route_coords: Vec<Coord> = Vec::new();
        let mut lights: Vec<[f64; 2]> = Vec::new();
        let mut legs = Vec::with_capacity(legs_paths.len());
        let (mut tot_dist, mut tot_weight, mut tot_duration) = (0.0, 0.0, 0.0);
        for lp in &legs_paths {
            let resolved: Vec<Resolved> = lp.pieces.iter().map(|p| resolve(self, &m, p)).collect();
            for c in self.signals_on(&m, &lp.pieces) {
                lights.push([c.lon(), c.lat()]);
            }
            let distance: f64 = resolved.iter().map(|r| r.distance).sum();
            let weight = lp.weight_ds as f64 / 10.0;
            // Real travel time (the weight may include avoid-area penalties).
            let duration: f64 = resolved.iter().map(|r| r.duration).sum();
            for r in &resolved {
                let skip = usize::from(!route_coords.is_empty());
                route_coords.extend_from_slice(&r.coords[skip.min(r.coords.len())..]);
            }
            // Summary: the two longest named roads, like OSRM.
            let mut by_name: Vec<(u32, f64)> = Vec::new();
            for r in &resolved {
                if r.name == crate::network::NO_NAME {
                    continue;
                }
                match by_name.iter_mut().find(|x| x.0 == r.name) {
                    Some(x) => x.1 += r.distance,
                    None => by_name.push((r.name, r.distance)),
                }
            }
            by_name.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
            let summary = by_name
                .iter()
                .take(2)
                .map(|x| self.graph.name(x.0))
                .collect::<Vec<_>>()
                .join(", ");
            let mut steps = if opts.steps { build_steps(self, &resolved, opts) } else { Vec::new() };
            // Make step durations add up exactly to the leg duration.
            let sum: f64 = steps.iter().map(|s| s.duration).sum();
            if sum > 0.0 {
                for s in steps.iter_mut() {
                    s.duration = round1(s.duration * duration / sum);
                }
            }
            tot_dist += distance;
            tot_weight += weight;
            tot_duration += duration;
            legs.push(Leg {
                distance: round1(distance),
                duration: round1(duration),
                weight: round1(weight),
                summary,
                steps,
            });
        }

        Ok(RouteResponse {
            code: "Ok",
            routes: vec![Route {
                distance: round1(tot_dist),
                duration: round1(tot_duration),
                weight: round1(tot_weight),
                weight_name: if opts.exclude.is_some() { "duration+avoid" } else { "duration" },
                geometry: opts.overview.then(|| make_geometry(&route_coords, opts.geometry)),
                legs,
                traffic_lights: lights,
            }],
            waypoints: snaps
                .iter()
                .map(|s| Waypoint {
                    location: [s.point.lon(), s.point.lat()],
                    name: self.graph.name(self.graph.segments[s.segment as usize].name).to_string(),
                    distance: round1(s.distance_m),
                })
                .collect(),
            metric_version: m.version,
        })
    }
}
