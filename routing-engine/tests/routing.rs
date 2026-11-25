//! End-to-end routing tests on small synthetic networks built in memory.

use routing_engine::build::build_graph;
use routing_engine::engine::Engine;
use routing_engine::geo::Coord;
use routing_engine::osm::profile::{car_profile, Profile};
use routing_engine::osm::{OsmData, OsmWay, Restriction, RestrictionKind, Via};
use routing_engine::route::RouteOptions;
use routing_engine::server::{apply, Selector, TrafficUpdate, UpdateRequest};

const SPACING: f64 = 0.002; // ~200 m

/// A `w` x `h` grid of two-way residential streets. Node (x, y) has OSM id `node(x, y)`;
/// every block edge is its own way so restrictions can address single turns.
struct Grid {
    w: i64,
    h: i64,
    osm: OsmData,
}

fn node(x: i64, y: i64) -> i64 {
    1000 + y * 100 + x
}

fn coord(x: i64, y: i64) -> Coord {
    Coord::from_deg(32.0 + y as f64 * SPACING, 34.8 + x as f64 * SPACING)
}

impl Grid {
    fn new(w: i64, h: i64) -> Grid {
        let mut osm = OsmData::default();
        for y in 0..h {
            for x in 0..w {
                osm.nodes.push((node(x, y), coord(x, y)));
            }
        }
        osm.nodes.sort_by_key(|n| n.0);
        let mut g = Grid { w, h, osm };
        for y in 0..h {
            for x in 0..w {
                if x + 1 < w {
                    g.add_way(node(x, y), node(x + 1, y), &[]);
                }
                if y + 1 < h {
                    g.add_way(node(x, y), node(x, y + 1), &[]);
                }
            }
        }
        g
    }

    fn way_id(a: i64, b: i64) -> i64 {
        a * 10_000 + b
    }

    fn add_way(&mut self, a: i64, b: i64, extra: &[(&'static str, &'static str)]) {
        let mut tags = vec![("highway", "residential"), ("name", "Street")];
        tags.extend_from_slice(extra);
        self.osm.ways.push(OsmWay {
            id: Self::way_id(a, b),
            refs: vec![a, b],
            profile: car_profile(tags.into_iter()).unwrap(),
        });
    }

    fn set_oneway(&mut self, a: i64, b: i64) {
        let id = Self::way_id(a, b);
        let w = self.osm.ways.iter_mut().find(|w| w.id == id).unwrap();
        w.profile.backward = false;
    }

    fn restrict(&mut self, from: (i64, i64), via: i64, to: (i64, i64), kind: RestrictionKind) {
        let wid = |(a, b): (i64, i64)| if a < b { Self::way_id(a, b) } else { Self::way_id(b, a) };
        self.osm.restrictions.push(Restriction {
            from_way: wid(from),
            via: Via::Node(via),
            to_way: wid(to),
            kind,
        });
    }

    fn engine(mut self) -> Engine {
        self.osm.ways.sort_by_key(|w| w.id);
        let _ = (self.w, self.h);
        Engine::new(build_graph(&self.osm, "test", Profile::Car))
    }
}

/// Midpoint between two grid nodes.
fn mid(a: (i64, i64), b: (i64, i64)) -> Coord {
    let (ca, cb) = (coord(a.0, a.1), coord(b.0, b.1));
    Coord::from_deg((ca.lat() + cb.lat()) / 2.0, (ca.lon() + cb.lon()) / 2.0)
}

/// Sequence of OSM ways a route uses (consecutive duplicates removed).
fn ways_of(engine: &Engine, from: Coord, to: Coord) -> Option<(Vec<i64>, f64)> {
    let m = engine.metric();
    let a = engine.snap(from)?;
    let b = engine.snap(to)?;
    let leg = engine.leg(&m, &a, &b)?;
    let mut ways: Vec<i64> = leg
        .pieces
        .iter()
        .map(|p| engine.graph.segment_way[p.segment as usize])
        .collect();
    ways.dedup();
    Some((ways, leg.weight_ds as f64 / 10.0))
}

#[test]
fn straight_route_on_grid() {
    let e = Grid::new(5, 5).engine();
    let (ways, secs) = ways_of(&e, mid((0, 2), (1, 2)), mid((3, 2), (4, 2))).unwrap();
    // Straight east along row 2: 3 blocks of ~188 m at 25 km/h ≈ 81 s.
    assert_eq!(ways.len(), 4, "{ways:?}");
    assert!((60.0..110.0).contains(&secs), "{secs}");
}

#[test]
fn no_left_turn_restriction_forces_detour() {
    // Drive east into the centre (2,2), want to go north to (2,4).
    let from = mid((0, 2), (1, 2));
    let to = mid((2, 3), (2, 4));
    let free = Grid::new(5, 5).engine();
    let (free_ways, free_secs) = ways_of(&free, from, to).unwrap();
    let turn_from = Grid::way_id(node(1, 2), node(2, 2));
    let turn_to = Grid::way_id(node(2, 2), node(2, 3));
    assert!(free_ways.windows(2).any(|w| w == [turn_from, turn_to]), "{free_ways:?}");

    let mut g = Grid::new(5, 5);
    g.restrict((node(1, 2), node(2, 2)), node(2, 2), (node(2, 2), node(2, 3)), RestrictionKind::No);
    let e = g.engine();
    assert_eq!(e.graph.meta.restrictions_applied, 1);
    let (ways, secs) = ways_of(&e, from, to).unwrap();
    assert!(
        !ways.windows(2).any(|w| w == [turn_from, turn_to]),
        "route still takes the forbidden left turn: {ways:?}"
    );
    assert!(secs > free_secs, "detour should be slower ({secs} vs {free_secs})");
}

#[test]
fn only_straight_restriction() {
    let mut g = Grid::new(5, 5);
    // Coming east into (2,2), only straight on to (3,2) is allowed.
    g.restrict((node(1, 2), node(2, 2)), node(2, 2), (node(2, 2), node(3, 2)), RestrictionKind::Only);
    let e = g.engine();
    let from = mid((0, 2), (1, 2));
    let to = mid((2, 1), (2, 0)); // south of the centre
    let (ways, _) = ways_of(&e, from, to).unwrap();
    let entry = Grid::way_id(node(1, 2), node(2, 2));
    let pos = ways.iter().position(|&w| w == entry);
    if let Some(p) = pos {
        if p + 1 < ways.len() {
            assert_eq!(ways[p + 1], Grid::way_id(node(2, 2), node(3, 2)), "{ways:?}");
        }
    }
}

#[test]
fn oneway_is_respected() {
    let mut g = Grid::new(4, 1);
    // A single street 0-1-2-3, with block 1->2 one-way eastbound.
    g.set_oneway(node(1, 0), node(2, 0));
    let e = g.engine();
    assert!(ways_of(&e, mid((0, 0), (1, 0)), mid((2, 0), (3, 0))).is_some());
    assert!(
        ways_of(&e, mid((2, 0), (3, 0)), mid((0, 0), (1, 0))).is_none(),
        "drove west against a one-way with no alternative"
    );
}

#[test]
fn closure_and_traffic_updates_change_the_route() {
    let e = Grid::new(5, 5).engine();
    let from = mid((0, 2), (1, 2));
    let to = mid((3, 2), (4, 2));
    let (before, t0) = ways_of(&e, from, to).unwrap();
    let blocked = Grid::way_id(node(2, 2), node(3, 2));
    assert!(before.contains(&blocked));

    let req = UpdateRequest {
        closures: vec![Selector {
            way_id: Some(blocked),
            near: None,
            radius_m: None,
            direction: None,
        }],
        ..Default::default()
    };
    let res = apply(&e, &req).unwrap();
    assert_eq!(res["metric_version"], 1);
    let (after, t1) = ways_of(&e, from, to).unwrap();
    assert!(!after.contains(&blocked), "{after:?}");
    assert!(t1 > t0);

    // Reopen it but make it very slow: the detour should still win.
    let req = UpdateRequest {
        reopen: vec![Selector {
            way_id: Some(blocked),
            near: None,
            radius_m: None,
            direction: None,
        }],
        traffic: vec![TrafficUpdate {
            at: Selector {
                way_id: Some(blocked),
                near: None,
                radius_m: None,
                direction: None,
            },
            speed_kmh: Some(2.0),
            factor: None,
        }],
        ..Default::default()
    };
    // Reopen is applied after traffic within one request, so send them separately.
    apply(
        &e,
        &UpdateRequest {
            reopen: req.reopen.clone(),
            ..Default::default()
        },
    )
    .unwrap();
    let (reopened, t2) = ways_of(&e, from, to).unwrap();
    assert!(reopened.contains(&blocked));
    assert!((t2 - t0).abs() < 0.5);
    apply(
        &e,
        &UpdateRequest {
            traffic: req.traffic.clone(),
            ..Default::default()
        },
    )
    .unwrap();
    let (slow, _) = ways_of(&e, from, to).unwrap();
    assert!(!slow.contains(&blocked), "{slow:?}");
}

#[test]
fn target_behind_start_on_oneway_requires_a_loop() {
    let mut g = Grid::new(3, 3);
    g.set_oneway(node(0, 0), node(1, 0));
    let e = g.engine();
    let a = coord(0, 0);
    let b = coord(1, 0);
    let p = |t: f64| Coord::from_deg(a.lat() + (b.lat() - a.lat()) * t, a.lon() + (b.lon() - a.lon()) * t);
    // Forward along the one-way: short.
    let (_, fwd) = ways_of(&e, p(0.2), p(0.8)).unwrap();
    // Backward: must go around a block.
    let (ways, back) = ways_of(&e, p(0.8), p(0.2)).unwrap();
    assert!(fwd < 30.0, "{fwd}");
    assert!(back > fwd * 3.0, "{back} vs {fwd}, {ways:?}");
}

#[test]
fn full_route_response_has_steps() {
    let e = Grid::new(5, 5).engine();
    let r = e
        .route(&[mid((0, 2), (1, 2)), mid((2, 3), (2, 4))], &RouteOptions::default())
        .unwrap();
    let route = &r.routes[0];
    assert!(route.distance > 300.0);
    let steps = &route.legs[0].steps;
    assert_eq!(steps.first().unwrap().maneuver.kind, "depart");
    assert_eq!(steps.last().unwrap().maneuver.kind, "arrive");
    assert!(
        steps.iter().any(|s| s.maneuver.kind == "turn" && s.maneuver.modifier == Some("left")),
        "{:#?}",
        steps.iter().map(|s| &s.maneuver.instruction).collect::<Vec<_>>()
    );
    let json = serde_json::to_value(&r).unwrap();
    assert_eq!(json["code"], "Ok");
}

#[test]
fn cch_matches_dijkstra_with_random_restrictions() {
    let mut g = Grid::new(12, 12);
    let mut seed = 42u64;
    let mut rnd = |n: i64| {
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        ((seed >> 33) % n as u64) as i64
    };
    for _ in 0..60 {
        let (x, y) = (1 + rnd(10), 1 + rnd(10));
        let via = node(x, y);
        let nbrs = [node(x - 1, y), node(x + 1, y), node(x, y - 1), node(x, y + 1)];
        let from = nbrs[rnd(4) as usize];
        let to = nbrs[rnd(4) as usize];
        if from != to {
            g.restrict((from, via), via, (via, to), RestrictionKind::No);
        }
    }
    for _ in 0..25 {
        let (x, y) = (rnd(11), rnd(12));
        g.set_oneway(node(x, y), node(x + 1, y));
    }
    let e = g.engine();
    assert!(e.graph.meta.restrictions_applied > 30);
    assert_eq!(routing_engine::verify::verify(&e, 40, 100, 7), 0);
}

#[test]
fn traffic_lights_counted_and_timing_updates_apply() {
    let mut g = Grid::new(5, 5);
    g.osm.signals = vec![(node(2, 2), routing_engine::osm::SIG_VEHICLE)];
    let e = g.engine();
    assert_eq!(e.graph.meta.num_signals, 1);
    let (a, b) = (mid((0, 2), (1, 2)), mid((3, 2), (4, 2)));
    let r = e.route(&[a, b], &RouteOptions::default()).unwrap();
    assert_eq!(r.routes[0].traffic_lights.len(), 1, "straight through the lit junction");
    let before = r.routes[0].duration;

    // A long red phase: 120 s cycle with 20 s green -> 120 * (100/120)^2 / 2 ≈ 41.7 s wait.
    let req: UpdateRequest = serde_json::from_value(serde_json::json!({
        "signals": [{ "osm_id": node(2, 2), "cycle_s": 120, "green_s": 20 }]
    }))
    .unwrap();
    let res = apply(&e, &req).unwrap();
    assert_eq!(res["active"]["signals"], 1);
    let after = e.route(&[a, b], &RouteOptions::default()).unwrap().routes[0].duration;
    // Either the route waits longer, or it detours around the light; both cost more time.
    assert!(after > before + 10.0, "{before} -> {after}");
}

#[test]
fn via_way_restriction_blocks_only_the_full_sequence() {
    // Way A = (0,2)-(1,2), via way V = (1,2)-(2,2), way B = (2,2)-(2,3).
    // "No A -> V -> B": coming from A you may drive through V but not turn onto B at its end.
    let from_a = mid((0, 2), (1, 2));
    let on_v = mid((1, 2), (2, 2));
    let target = mid((2, 3), (2, 4));
    let a = Grid::way_id(node(0, 2), node(1, 2));
    let v = Grid::way_id(node(1, 2), node(2, 2));
    let b = Grid::way_id(node(2, 2), node(2, 3));

    let mut g = Grid::new(5, 5);
    g.osm.restrictions.push(Restriction {
        from_way: a,
        via: Via::Ways(vec![v]),
        to_way: b,
        kind: RestrictionKind::No,
    });
    let e = g.engine();
    assert_eq!(e.graph.meta.restrictions_applied, 1);

    let (ways, _) = ways_of(&e, from_a, target).unwrap();
    assert!(!ways.windows(3).any(|w| w == [a, v, b]), "took the forbidden A-V-B: {ways:?}");
    // Starting on V itself (not coming from A) the turn onto B is allowed.
    let (ways, _) = ways_of(&e, on_v, target).unwrap();
    assert!(ways.windows(2).any(|w| w == [v, b]), "V -> B should be allowed: {ways:?}");
    // CCH still exact with the duplicated nodes.
    assert_eq!(routing_engine::verify::verify(&e, 20, 100, 3), 0);
}

#[test]
fn walking_graph_is_node_based_and_exact() {
    let mut g = Grid::new(12, 12);
    g.osm.signals = vec![(node(3, 3), routing_engine::osm::SIG_VEHICLE), (node(6, 6), routing_engine::osm::SIG_VEHICLE)];
    for x in 0..6 {
        g.set_oneway(node(x, 4), node(x + 1, 4));
    }
    g.osm.ways.sort_by_key(|w| w.id);
    let e = Engine::new(build_graph(&g.osm, "test", Profile::Foot));
    assert!(e.graph.meta.node_based);
    // Junction graph: one node per junction, one arc per allowed segment direction.
    assert_eq!(e.graph.first_out.len() - 1, e.graph.meta.num_junctions);
    assert_eq!(routing_engine::verify::verify(&e, 30, 144, 11), 0);

    // Same segment, target ahead: walks straight there (~60% of a ~188 m block at 25 km/h
    // in this car-tagged fixture).
    let (a, b) = (coord(0, 0), coord(1, 0));
    let p = |t: f64| Coord::from_deg(a.lat() + (b.lat() - a.lat()) * t, a.lon() + (b.lon() - a.lon()) * t);
    let r = e.route(&[p(0.2), p(0.8)], &RouteOptions::default()).unwrap();
    assert!((r.routes[0].distance - 0.6 * 188.0).abs() < 15.0, "{}", r.routes[0].distance);
    let back = e.route(&[p(0.8), p(0.2)], &RouteOptions::default()).unwrap();
    assert!((back.routes[0].distance - r.routes[0].distance).abs() < 1.0);

    // Across the grid, with steps and lights.
    let r = e.route(&[mid((0, 0), (1, 0)), mid((10, 11), (11, 11))], &RouteOptions::default()).unwrap();
    assert_eq!(r.routes[0].legs[0].steps.last().unwrap().maneuver.kind, "arrive");
    assert_eq!(r.routes[0].legs[0].steps[0].mode, "walking");
}

#[test]
fn one_wait_per_intersection_and_signal_modes() {
    // A dual-carriageway-style intersection: lights on two neighbouring junction nodes 40 m
    // apart (both mapped as traffic_signals). Passing both must cost one wait, not two.
    let mut g = Grid::new(6, 3);
    // Squeeze column 3 next to column 2 so (2,1) and (3,1) are one intersection (~20 m).
    for n in g.osm.nodes.iter_mut() {
        let (x, y) = ((n.0 - 1000) % 100, (n.0 - 1000) / 100);
        if x >= 3 {
            n.1 = Coord::from_deg(32.0 + y as f64 * SPACING, 34.8 + (x as f64 - 0.9) * SPACING);
        }
    }
    g.osm.signals = vec![
        (node(2, 1), routing_engine::osm::SIG_VEHICLE),
        (node(3, 1), routing_engine::osm::SIG_VEHICLE),
    ];
    let e = g.engine();
    let route = |e: &Engine| e.route(&[mid((0, 1), (1, 1)), mid((4, 1), (5, 1))], &RouteOptions::default()).unwrap();
    let r = route(&e);
    assert_eq!(r.routes[0].traffic_lights.len(), 1, "two lights of one intersection count once");
    let with_lights = r.routes[0].duration;

    let set = |mode: &str| {
        let req: UpdateRequest = serde_json::from_value(serde_json::json!({ "signal_mode": mode })).unwrap();
        apply(&e, &req).unwrap();
    };
    set("off");
    let off = route(&e);
    assert!(off.routes[0].traffic_lights.is_empty());
    // One intersection wait (12 s), not two.
    assert!((with_lights - off.routes[0].duration - 12.0).abs() < 1.0, "{with_lights} vs {}", off.routes[0].duration);

    set("timed-only");
    assert!(route(&e).routes[0].traffic_lights.is_empty(), "untimed lights are ignored");
    let req: UpdateRequest =
        serde_json::from_value(serde_json::json!({ "signals": [{ "osm_id": node(2, 1), "delay_s": 30 }] })).unwrap();
    apply(&e, &req).unwrap();
    let timed = route(&e);
    assert_eq!(timed.routes[0].traffic_lights.len(), 1);
    assert!((timed.routes[0].duration - off.routes[0].duration - 30.0).abs() < 1.0);
}
