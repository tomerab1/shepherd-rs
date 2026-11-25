//! Regression routes on the real Israel graph. Skipped unless `data/il.graph` exists
//! (build it with `shepherd preprocess data/il-car-only.osm.pbf data/il.graph`).

use routing_engine::engine::Engine;
use routing_engine::geo::Coord;
use routing_engine::graph::Graph;
use routing_engine::route::RouteOptions;
use std::path::Path;
use std::sync::OnceLock;

fn engine() -> Option<&'static Engine> {
    static E: OnceLock<Option<Engine>> = OnceLock::new();
    E.get_or_init(|| {
        let p = Path::new(env!("CARGO_MANIFEST_DIR")).join("data/il.graph");
        p.exists().then(|| Engine::new(Graph::load(&p).unwrap()))
    })
    .as_ref()
}

fn ll(lat: f64, lon: f64) -> Coord {
    Coord::from_deg(lat, lon)
}

/// (name, from, to, distance km range, duration min range)
const CASES: &[(&str, (f64, f64), (f64, f64), (f64, f64), (f64, f64))] = &[
    ("Tel Aviv -> Jerusalem", (32.0744, 34.7918), (31.7767, 35.2276), (55.0, 75.0), (40.0, 80.0)),
    ("Haifa -> Tel Aviv", (32.7940, 34.9896), (32.0853, 34.7818), (85.0, 110.0), (55.0, 100.0)),
    ("Beer Sheva -> Eilat", (31.2520, 34.7915), (29.5577, 34.9519), (220.0, 270.0), (140.0, 240.0)),
    ("Short hop in Tel Aviv", (32.0809, 34.7806), (32.0731, 34.7748), (0.8, 3.0), (2.0, 12.0)),
];

#[test]
fn known_routes_are_plausible() {
    let Some(e) = engine() else {
        eprintln!("skipping: data/il.graph not built");
        return;
    };
    for (name, a, b, km, min) in CASES {
        let r = e
            .route(&[ll(a.0, a.1), ll(b.0, b.1)], &RouteOptions::default())
            .unwrap_or_else(|err| panic!("{name}: {err}"));
        let route = &r.routes[0];
        let (d, t) = (route.distance / 1000.0, route.duration / 60.0);
        assert!((km.0..km.1).contains(&d), "{name}: {d:.1} km not in {km:?}");
        assert!((min.0..min.1).contains(&t), "{name}: {t:.1} min not in {min:?}");
        let steps = &route.legs[0].steps;
        assert_eq!(steps.first().unwrap().maneuver.kind, "depart");
        assert_eq!(steps.last().unwrap().maneuver.kind, "arrive");
    }
}

#[test]
fn reverse_routes_are_similar() {
    let Some(e) = engine() else { return };
    let (a, b) = (ll(32.0744, 34.7918), ll(31.7767, 35.2276));
    let o = RouteOptions::default();
    let there = e.route(&[a, b], &o).unwrap().routes[0].duration;
    let back = e.route(&[b, a], &o).unwrap().routes[0].duration;
    assert!((there - back).abs() / there < 0.25, "{there} vs {back}");
}

#[test]
fn cch_matches_dijkstra_on_israel() {
    let Some(e) = engine() else { return };
    assert_eq!(routing_engine::verify::verify(e, 5, 200, 99), 0);
}
