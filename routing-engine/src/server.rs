//! HTTP API (axum). OSRM-compatible `route` and `nearest` services plus admin endpoints for
//! live traffic and closures.
//!
//! - `GET  /route/v1/driving/{lon,lat;lon,lat;...}?steps=true&overview=full&geometries=polyline`
//! - `GET  /nearest/v1/driving/{lon,lat}`
//! - `GET  /health`, `GET /metrics` (Prometheus text format)
//! - `GET|POST|DELETE /admin/updates` — list, apply, or clear traffic/closure overrides.
//!   Protected by `x-admin-token` when `SHEPHERD_ADMIN_TOKEN` is set.
//!
//! Route computation runs on the blocking pool, bounded by a semaphore so a burst of requests
//! queues instead of oversubscribing the CPU.

use crate::engine::{Engine, Overrides, RouteError};
use crate::geo::Coord;
use crate::graph::Graph;
use crate::route::{GeometryFormat, RouteOptions};
use arc_swap::ArcSwap;
use axum::extract::{Path, Query, State};
use axum::http::{HeaderMap, StatusCode};
use axum::response::{IntoResponse, Response};
use axum::routing::get;
use axum::{Json, Router};
use serde::Deserialize;
use serde_json::json;
use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Instant;
use tokio::sync::Semaphore;

pub const MAX_WAYPOINTS: usize = 25;

const BUCKETS_MS: [f64; 10] = [1.0, 2.0, 5.0, 10.0, 25.0, 50.0, 100.0, 250.0, 500.0, 1000.0];

#[derive(Default)]
pub struct Stats {
    route_ok: AtomicU64,
    route_err: AtomicU64,
    nearest: AtomicU64,
    updates: AtomicU64,
    latency_us_sum: AtomicU64,
    buckets: [AtomicU64; 10],
}

/// One loaded graph (one routing profile).
pub struct Slot {
    pub profile: String,
    pub engine: ArcSwap<Engine>,
    pub graph_path: PathBuf,
}

pub struct AppState {
    pub slots: Vec<Slot>,
    pub areas: Vec<(String, crate::area::Area)>,
    pub admin_token: Option<String>,
    pub limiter: Semaphore,
    pub stats: Stats,
    pub started: Instant,
}

pub fn router(state: Arc<AppState>) -> Router {
    Router::new()
        .route("/route/v1/{profile}/{coords}", get(route_handler))
        .route("/nearest/v1/{profile}/{coord}", get(nearest_handler))
        .route("/health", get(health))
        .route("/metrics", get(metrics))
        .route(
            "/admin/updates",
            get(list_updates).post(apply_updates).delete(clear_updates),
        )
        .with_state(state)
}

impl AppState {
    /// The slot for a profile name such as `driving`, `car`, `foot` or `walking`.
    fn slot(&self, name: &str) -> Option<&Slot> {
        let wanted = crate::osm::profile::Profile::parse(name)?.name();
        self.slots.iter().find(|s| s.profile == wanted)
    }
}

fn unknown_profile(name: &str, st: &AppState) -> Response {
    let have: Vec<&str> = st.slots.iter().map(|s| s.profile.as_str()).collect();
    osrm_error(
        StatusCode::BAD_REQUEST,
        "InvalidProfile",
        format!("profile '{name}' is not loaded (available: {})", have.join(", ")),
    )
}

fn osrm_error(status: StatusCode, code: &str, message: impl std::fmt::Display) -> Response {
    (status, Json(json!({ "code": code, "message": message.to_string() }))).into_response()
}

fn parse_coords(s: &str) -> Result<Vec<Coord>, String> {
    let s = s.strip_suffix(".json").unwrap_or(s);
    s.split(';')
        .map(|pair| {
            let (lon, lat) = pair.split_once(',').ok_or_else(|| format!("bad coordinate '{pair}'"))?;
            let lon: f64 = lon.trim().parse().map_err(|_| format!("bad longitude '{lon}'"))?;
            let lat: f64 = lat.trim().parse().map_err(|_| format!("bad latitude '{lat}'"))?;
            if !(-180.0..=180.0).contains(&lon) || !(-90.0..=90.0).contains(&lat) {
                return Err(format!("coordinate out of range '{pair}'"));
            }
            Ok(Coord::from_deg(lat, lon))
        })
        .collect()
}

#[derive(Deserialize, Default)]
struct RouteParams {
    steps: Option<bool>,
    overview: Option<String>,
    geometries: Option<String>,
    /// Area to avoid, e.g. `exclude=judea-samaria`.
    exclude: Option<String>,
}

async fn route_handler(
    State(st): State<Arc<AppState>>,
    Path((profile, coords)): Path<(String, String)>,
    Query(p): Query<RouteParams>,
) -> Response {
    let Some(slot) = st.slot(&profile) else {
        return unknown_profile(&profile, &st);
    };
    let coords = match parse_coords(&coords) {
        Ok(c) => c,
        Err(e) => return osrm_error(StatusCode::BAD_REQUEST, "InvalidQuery", e),
    };
    if coords.len() > MAX_WAYPOINTS {
        return osrm_error(StatusCode::BAD_REQUEST, "TooBig", format!("at most {MAX_WAYPOINTS} waypoints"));
    }
    let geometry = match p.geometries.as_deref() {
        None | Some("polyline") => GeometryFormat::Polyline,
        Some("polyline6") => GeometryFormat::Polyline6,
        Some("geojson") => GeometryFormat::GeoJson,
        Some(o) => return osrm_error(StatusCode::BAD_REQUEST, "InvalidOptions", format!("unknown geometries '{o}'")),
    };
    let opts = RouteOptions {
        steps: p.steps.unwrap_or(false),
        overview: !matches!(p.overview.as_deref(), Some("false")),
        geometry,
        exclude: p.exclude.filter(|e| !e.is_empty()),
    };

    let _permit = st.limiter.acquire().await.expect("semaphore closed");
    let t = Instant::now();
    let engine = slot.engine.load_full();
    let res = tokio::task::spawn_blocking(move || engine.route(&coords, &opts)).await;
    let us = t.elapsed().as_micros() as u64;
    st.stats.latency_us_sum.fetch_add(us, Ordering::Relaxed);
    let ms = us as f64 / 1000.0;
    for (i, b) in BUCKETS_MS.iter().enumerate() {
        if ms <= *b {
            st.stats.buckets[i].fetch_add(1, Ordering::Relaxed);
        }
    }
    match res {
        Ok(Ok(r)) => {
            st.stats.route_ok.fetch_add(1, Ordering::Relaxed);
            Json(r).into_response()
        }
        Ok(Err(e)) => {
            st.stats.route_err.fetch_add(1, Ordering::Relaxed);
            let code = match e {
                RouteError::NoSegment(_) => "NoSegment",
                RouteError::NoRoute(..) => "NoRoute",
                RouteError::InvalidQuery(_) => "InvalidQuery",
            };
            osrm_error(StatusCode::BAD_REQUEST, code, e)
        }
        Err(e) => {
            st.stats.route_err.fetch_add(1, Ordering::Relaxed);
            osrm_error(StatusCode::INTERNAL_SERVER_ERROR, "InternalError", e)
        }
    }
}

async fn nearest_handler(State(st): State<Arc<AppState>>, Path((profile, coord)): Path<(String, String)>) -> Response {
    let Some(slot) = st.slot(&profile) else {
        return unknown_profile(&profile, &st);
    };
    let c = match parse_coords(&coord) {
        Ok(c) if c.len() == 1 => c[0],
        Ok(_) => return osrm_error(StatusCode::BAD_REQUEST, "InvalidQuery", "expected one coordinate"),
        Err(e) => return osrm_error(StatusCode::BAD_REQUEST, "InvalidQuery", e),
    };
    st.stats.nearest.fetch_add(1, Ordering::Relaxed);
    let engine = slot.engine.load();
    match engine.snap(c) {
        Some(s) => {
            let g = &engine.graph;
            let seg = &g.segments[s.segment as usize];
            Json(json!({
                "code": "Ok",
                "waypoints": [{
                    "location": [s.point.lon(), s.point.lat()],
                    "distance": (s.distance_m * 10.0).round() / 10.0,
                    "name": g.name(seg.name),
                    "way_id": g.segment_way[s.segment as usize],
                    "segment": s.segment,
                }]
            }))
            .into_response()
        }
        None => osrm_error(StatusCode::BAD_REQUEST, "NoSegment", "no road nearby"),
    }
}

async fn health(State(st): State<Arc<AppState>>) -> Response {
    let profiles: Vec<serde_json::Value> = st
        .slots
        .iter()
        .map(|s| {
            let e = s.engine.load();
            let m = e.metric();
            json!({
                "profile": s.profile,
                "avoid_areas": e.area_names(),
                "graph": &e.graph.meta,
                "metric_version": m.version,
                "customize_ms": m.customize_ms,
            })
        })
        .collect();
    // Top-level fields describe the first profile, for simple clients.
    let first = &profiles[0];
    Json(json!({
        "status": "ok",
        "uptime_s": st.started.elapsed().as_secs(),
        "graph": first["graph"],
        "metric_version": first["metric_version"],
        "customize_ms": first["customize_ms"],
        "profiles": profiles,
    }))
    .into_response()
}

async fn metrics(State(st): State<Arc<AppState>>) -> Response {
    let s = &st.stats;
    let mut out = String::new();
    let ok = s.route_ok.load(Ordering::Relaxed);
    let err = s.route_err.load(Ordering::Relaxed);
    out += "# TYPE shepherd_route_requests_total counter\n";
    out += &format!("shepherd_route_requests_total{{result=\"ok\"}} {ok}\n");
    out += &format!("shepherd_route_requests_total{{result=\"error\"}} {err}\n");
    out += "# TYPE shepherd_nearest_requests_total counter\n";
    out += &format!("shepherd_nearest_requests_total {}\n", s.nearest.load(Ordering::Relaxed));
    out += "# TYPE shepherd_metric_updates_total counter\n";
    out += &format!("shepherd_metric_updates_total {}\n", s.updates.load(Ordering::Relaxed));
    out += "# TYPE shepherd_route_latency_seconds histogram\n";
    for (i, b) in BUCKETS_MS.iter().enumerate() {
        out += &format!(
            "shepherd_route_latency_seconds_bucket{{le=\"{}\"}} {}\n",
            b / 1000.0,
            s.buckets[i].load(Ordering::Relaxed)
        );
    }
    out += &format!("shepherd_route_latency_seconds_bucket{{le=\"+Inf\"}} {}\n", ok + err);
    out += &format!(
        "shepherd_route_latency_seconds_sum {}\n",
        s.latency_us_sum.load(Ordering::Relaxed) as f64 / 1e6
    );
    out += &format!("shepherd_route_latency_seconds_count {}\n", ok + err);
    out += "# TYPE shepherd_metric_version gauge\n";
    for slot in &st.slots {
        out += &format!(
            "shepherd_metric_version{{profile=\"{}\"}} {}\n",
            slot.profile,
            slot.engine.load().metric().version
        );
    }
    (StatusCode::OK, [("content-type", "text/plain; version=0.0.4")], out).into_response()
}

fn check_admin(st: &AppState, headers: &HeaderMap) -> Result<(), Response> {
    match &st.admin_token {
        None => Ok(()),
        Some(t) if headers.get("x-admin-token").and_then(|v| v.to_str().ok()) == Some(t.as_str()) => Ok(()),
        Some(_) => Err(osrm_error(StatusCode::UNAUTHORIZED, "Unauthorized", "missing or wrong x-admin-token")),
    }
}

/// Which segments an update applies to.
#[derive(Deserialize, Debug, Clone)]
pub struct Selector {
    pub way_id: Option<i64>,
    /// `[lon, lat]`: every road within `radius_m` (default 25 m).
    pub near: Option<[f64; 2]>,
    pub radius_m: Option<f64>,
    /// `both` (default), `forward` or `backward` relative to the OSM way direction.
    pub direction: Option<String>,
}

#[derive(Deserialize, Debug, Clone)]
pub struct TrafficUpdate {
    #[serde(flatten)]
    pub at: Selector,
    /// Absolute travel speed in km/h (0 closes the road in that direction).
    pub speed_kmh: Option<f32>,
    /// Multiplier on free-flow travel time, e.g. 2.5 for heavy congestion.
    pub factor: Option<f32>,
}

/// Timing for traffic lights, selected by OSM node id or by location.
#[derive(Deserialize, Debug, Clone)]
pub struct SignalUpdate {
    pub osm_id: Option<i64>,
    /// `[lon, lat]`: every traffic light within `radius_m` (default 30 m).
    pub near: Option<[f64; 2]>,
    pub radius_m: Option<f64>,
    /// Expected wait in seconds, if known directly.
    pub delay_s: Option<f32>,
    /// Or signal timing: cycle length and effective green time (s) for the approach.
    pub cycle_s: Option<f32>,
    pub green_s: Option<f32>,
}

impl SignalUpdate {
    /// Expected wait: given directly, or Webster's uniform delay C·(1−g/C)²/2.
    pub fn delay(&self) -> Result<f32, String> {
        if let Some(d) = self.delay_s {
            return Ok(d.max(0.0));
        }
        match (self.cycle_s, self.green_s) {
            (Some(c), Some(g)) if c > 0.0 && g >= 0.0 && g <= c => {
                let r = 1.0 - g / c;
                Ok(c * r * r / 2.0)
            }
            _ => Err("signal update needs delay_s, or cycle_s and green_s (0 <= green <= cycle)".into()),
        }
    }
}

#[derive(Deserialize, Debug, Default)]
pub struct UpdateRequest {
    #[serde(default)]
    pub reset: bool,
    #[serde(default)]
    pub traffic: Vec<TrafficUpdate>,
    #[serde(default)]
    pub closures: Vec<Selector>,
    #[serde(default)]
    pub reopen: Vec<Selector>,
    #[serde(default)]
    pub signals: Vec<SignalUpdate>,
}

fn select(g: &Graph, s: &Selector) -> Result<(Vec<u32>, [bool; 2]), String> {
    let dirs = match s.direction.as_deref() {
        None | Some("both") => [true, true],
        Some("forward") => [true, false],
        Some("backward") => [false, true],
        Some(o) => return Err(format!("unknown direction '{o}'")),
    };
    let segs = match (s.way_id, s.near) {
        (Some(w), _) => g.segments_of_way(w).collect(),
        (None, Some([lon, lat])) => g.segments_near(Coord::from_deg(lat, lon), s.radius_m.unwrap_or(25.0).min(5_000.0)),
        (None, None) => return Err("selector needs way_id or near".into()),
    };
    Ok((segs, dirs))
}

/// Applies an update request to the engine. Shared by the HTTP handler and tests.
pub fn apply(engine: &Engine, req: &UpdateRequest) -> Result<serde_json::Value, String> {
    let g = &engine.graph;
    let mut traffic = Vec::new();
    for t in &req.traffic {
        let (segs, dirs) = select(g, &t.at)?;
        if t.speed_kmh.is_none() && t.factor.is_none() {
            return Err("traffic update needs speed_kmh or factor".into());
        }
        traffic.push((segs, dirs, t.speed_kmh, t.factor));
    }
    let closures = req.closures.iter().map(|s| select(g, s).map(|x| x.0)).collect::<Result<Vec<_>, _>>()?;
    let reopen = req.reopen.iter().map(|s| select(g, s).map(|x| x.0)).collect::<Result<Vec<_>, _>>()?;
    let mut signals: Vec<(u32, f32)> = Vec::new();
    for u in &req.signals {
        let delay = u.delay()?;
        let hits: Vec<u32> = match (u.osm_id, u.near) {
            (Some(id), _) => (0..g.signals.len() as u32).filter(|&i| g.signals[i as usize].osm_id == id).collect(),
            (None, Some([lon, lat])) => {
                let c = Coord::from_deg(lat, lon);
                let r = u.radius_m.unwrap_or(30.0);
                (0..g.signals.len() as u32)
                    .filter(|&i| crate::geo::haversine(g.signals[i as usize].coord, c) <= r)
                    .collect()
            }
            (None, None) => return Err("signal update needs osm_id or near".into()),
        };
        signals.extend(hits.into_iter().map(|i| (i, delay)));
    }
    let mut affected = 0usize;
    let (version, ms) = engine.update(|o: &mut Overrides| {
        if req.reset {
            *o = Overrides::default();
        }
        for (segs, dirs, speed, factor) in &traffic {
            for &s in segs {
                affected += 1;
                for k in 0..2 {
                    if dirs[k] {
                        if let Some(v) = speed {
                            o.speed.entry(s).or_default()[k] = Some(*v);
                        }
                        if let Some(f) = factor {
                            o.factor.entry(s).or_default()[k] = Some(*f);
                        }
                    }
                }
            }
        }
        for segs in &closures {
            affected += segs.len();
            o.closed.extend(segs.iter().copied());
        }
        for &(i, d) in &signals {
            affected += 1;
            o.signal_delay.insert(i, d);
        }
        for segs in &reopen {
            affected += segs.len();
            for s in segs {
                o.closed.remove(s);
                o.speed.remove(s);
                o.factor.remove(s);
            }
        }
    });
    let o = engine.overrides();
    Ok(json!({
        "metric_version": version,
        "customize_ms": (ms * 10.0).round() / 10.0,
        "segments_affected": affected,
        "active": {
            "speed": o.speed.len(),
            "factor": o.factor.len(),
            "closed": o.closed.len(),
            "signals": o.signal_delay.len(),
        },
    }))
}

#[derive(Deserialize, Default)]
struct AdminParams {
    profile: Option<String>,
}

fn admin_slot<'a>(st: &'a AppState, p: &AdminParams) -> Result<&'a Slot, Response> {
    let name = p.profile.as_deref().unwrap_or("driving");
    st.slot(name).ok_or_else(|| unknown_profile(name, st))
}

async fn apply_updates(
    State(st): State<Arc<AppState>>,
    headers: HeaderMap,
    Query(p): Query<AdminParams>,
    Json(req): Json<UpdateRequest>,
) -> Response {
    if let Err(r) = check_admin(&st, &headers) {
        return r;
    }
    let engine = match admin_slot(&st, &p) {
        Ok(s) => s.engine.load_full(),
        Err(r) => return r,
    };
    match tokio::task::spawn_blocking(move || apply(&engine, &req)).await {
        Ok(Ok(v)) => {
            st.stats.updates.fetch_add(1, Ordering::Relaxed);
            Json(v).into_response()
        }
        Ok(Err(e)) => osrm_error(StatusCode::BAD_REQUEST, "InvalidUpdate", e),
        Err(e) => osrm_error(StatusCode::INTERNAL_SERVER_ERROR, "InternalError", e),
    }
}

async fn clear_updates(State(st): State<Arc<AppState>>, headers: HeaderMap, Query(p): Query<AdminParams>) -> Response {
    if let Err(r) = check_admin(&st, &headers) {
        return r;
    }
    let engine = match admin_slot(&st, &p) {
        Ok(s) => s.engine.load_full(),
        Err(r) => return r,
    };
    let req = UpdateRequest {
        reset: true,
        ..Default::default()
    };
    match tokio::task::spawn_blocking(move || apply(&engine, &req)).await {
        Ok(Ok(v)) => Json(v).into_response(),
        Ok(Err(e)) => osrm_error(StatusCode::BAD_REQUEST, "InvalidUpdate", e),
        Err(e) => osrm_error(StatusCode::INTERNAL_SERVER_ERROR, "InternalError", e),
    }
}

async fn list_updates(State(st): State<Arc<AppState>>, headers: HeaderMap, Query(p): Query<AdminParams>) -> Response {
    if let Err(r) = check_admin(&st, &headers) {
        return r;
    }
    let engine = match admin_slot(&st, &p) {
        Ok(s) => s.engine.load(),
        Err(r) => return r,
    };
    let o = engine.overrides();
    let g = &engine.graph;
    let way = |s: &u32| g.segment_way[*s as usize];
    let mut ways: HashMap<i64, usize> = HashMap::new();
    for s in o.closed.iter() {
        *ways.entry(way(s)).or_default() += 1;
    }
    Json(json!({
        "metric_version": engine.metric().version,
        "closed_segments": o.closed.len(),
        "closed_ways": ways.keys().collect::<Vec<_>>(),
        "speed_overrides": o.speed.len(),
        "factor_overrides": o.factor.len(),
    }))
    .into_response()
}

/// Loads the graph and builds an engine (customizing the free-flow metric).
pub fn load_engine(path: &std::path::Path, areas: &[(String, crate::area::Area)]) -> anyhow::Result<Engine> {
    let t = Instant::now();
    let graph = Graph::load(path)?;
    tracing::info!(
        segments = graph.meta.num_segments,
        ebg_nodes = graph.meta.num_ebg_nodes,
        cch_edges = graph.meta.num_cch_edges,
        "mapped {} in {:.1?}",
        path.display(),
        t.elapsed()
    );
    Ok(Engine::with_areas(graph, areas.to_vec()))
}

pub async fn serve(
    graph_paths: Vec<PathBuf>,
    areas: Vec<(String, crate::area::Area)>,
    bind: String,
    max_concurrent: usize,
) -> anyhow::Result<()> {
    let mut slots: Vec<Slot> = Vec::new();
    for path in graph_paths {
        let engine = load_engine(&path, &areas)?;
        let profile = engine.graph.meta.profile.clone();
        if slots.iter().any(|s| s.profile == profile) {
            anyhow::bail!("two graphs for profile '{profile}'");
        }
        tracing::info!("profile '{profile}' ready from {}", path.display());
        slots.push(Slot {
            profile,
            engine: ArcSwap::from_pointee(engine),
            graph_path: path,
        });
    }
    if slots.is_empty() {
        anyhow::bail!("no graph given");
    }
    let state = Arc::new(AppState {
        slots,
        areas,
        admin_token: std::env::var("SHEPHERD_ADMIN_TOKEN").ok().filter(|s| !s.is_empty()),
        limiter: Semaphore::new(max_concurrent.max(1)),
        stats: Stats::default(),
        started: Instant::now(),
    });

    // SIGHUP: reload every graph file (e.g. after a nightly OSM refresh) without downtime.
    #[cfg(unix)]
    {
        let st = state.clone();
        tokio::spawn(async move {
            let mut hup = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::hangup()).unwrap();
            while hup.recv().await.is_some() {
                for (i, slot) in st.slots.iter().enumerate() {
                    let path = slot.graph_path.clone();
                    let areas = st.areas.clone();
                    match tokio::task::spawn_blocking(move || load_engine(&path, &areas)).await {
                        Ok(Ok(e)) => {
                            st.slots[i].engine.store(Arc::new(e));
                            tracing::info!("reloaded '{}'; live overrides were cleared", slot.profile);
                        }
                        Ok(Err(e)) => tracing::error!("reload of '{}' failed, keeping old graph: {e:#}", slot.profile),
                        Err(e) => tracing::error!("reload task failed: {e}"),
                    }
                }
            }
        });
    }

    let listener = tokio::net::TcpListener::bind(&bind).await?;
    tracing::info!("listening on http://{}", listener.local_addr()?);
    axum::serve(listener, router(state))
        .with_graceful_shutdown(shutdown_signal())
        .await?;
    tracing::info!("shut down");
    Ok(())
}

async fn shutdown_signal() {
    let ctrl_c = async {
        let _ = tokio::signal::ctrl_c().await;
    };
    #[cfg(unix)]
    let term = async {
        if let Ok(mut s) = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate()) {
            s.recv().await;
        }
    };
    #[cfg(not(unix))]
    let term = std::future::pending::<()>();
    tokio::select! {
        _ = ctrl_c => {},
        _ = term => {},
    }
}
