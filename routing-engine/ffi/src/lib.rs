//! C interface to the routing engine, so an app can route on-device without a server.
//!
//! Every call that returns a `char *` returns JSON in the same shape as the HTTP API
//! (`/route/v1`, `/lights/v1`), or `{"code": "...", "message": "..."}` on error. Free the
//! string with `shepherd_string_free`. Handles are thread-safe: routes may run concurrently.

use routing_engine::area::Area;
use routing_engine::engine::{Engine, RouteError};
use routing_engine::geo::Coord;
use routing_engine::graph::Graph;
use routing_engine::route::{GeometryFormat, RouteOptions};
use routing_engine::signals::{lights_in, SignalMode};
use serde_json::json;
use std::ffi::{c_char, CStr, CString};
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::path::Path;

pub struct ShepherdEngine(Engine);

fn to_c(v: serde_json::Value) -> *mut c_char {
    CString::new(v.to_string()).map_or(std::ptr::null_mut(), CString::into_raw)
}

fn err(code: &str, message: impl ToString) -> *mut c_char {
    to_c(json!({ "code": code, "message": message.to_string() }))
}

unsafe fn str_arg<'a>(p: *const c_char) -> Option<&'a str> {
    if p.is_null() {
        return None;
    }
    CStr::from_ptr(p).to_str().ok().filter(|s| !s.is_empty())
}

/// Opens a graph file (memory-mapped) and builds its metric. `avoid_area_name` and
/// `avoid_area_geojson` (both optional) register an area that routes can exclude.
/// Returns NULL on failure and, if `error_out` is not NULL, an error string there.
///
/// # Safety
/// Strings must be NUL-terminated UTF-8 or NULL.
#[no_mangle]
pub unsafe extern "C" fn shepherd_open(
    graph_path: *const c_char,
    avoid_area_name: *const c_char,
    avoid_area_geojson: *const c_char,
    error_out: *mut *mut c_char,
) -> *mut ShepherdEngine {
    let res = catch_unwind(AssertUnwindSafe(|| -> anyhow_like::Result<Engine> {
        let path = str_arg(graph_path).ok_or("graph path is required")?;
        let graph = Graph::load(Path::new(path)).map_err(|e| format!("{e:#}"))?;
        let mut areas = Vec::new();
        if let (Some(name), Some(file)) = (str_arg(avoid_area_name), str_arg(avoid_area_geojson)) {
            let area = Area::from_geojson(Path::new(file)).map_err(|e| format!("{e:#}"))?;
            areas.push((name.to_string(), area));
        }
        Ok(Engine::with_areas(graph, areas))
    }));
    let msg = match res {
        Ok(Ok(e)) => return Box::into_raw(Box::new(ShepherdEngine(e))),
        Ok(Err(m)) => m,
        Err(_) => "engine panicked while loading".to_string(),
    };
    if !error_out.is_null() {
        *error_out = CString::new(msg).map_or(std::ptr::null_mut(), CString::into_raw);
    }
    std::ptr::null_mut()
}

/// Minimal string-error result, to keep the FFI crate free of extra dependencies.
mod anyhow_like {
    pub type Result<T> = std::result::Result<T, String>;
}

/// # Safety
/// `e` must come from `shepherd_open` and not be used afterwards.
#[no_mangle]
pub unsafe extern "C" fn shepherd_close(e: *mut ShepherdEngine) {
    if !e.is_null() {
        drop(Box::from_raw(e));
    }
}

/// # Safety
/// `s` must come from this library (or be NULL).
#[no_mangle]
pub unsafe extern "C" fn shepherd_string_free(s: *mut c_char) {
    if !s.is_null() {
        drop(CString::from_raw(s));
    }
}

/// Basic facts about the loaded graph, as JSON.
///
/// # Safety
/// `e` must be a live handle.
#[no_mangle]
pub unsafe extern "C" fn shepherd_info(e: *const ShepherdEngine) -> *mut c_char {
    let Some(e) = e.as_ref() else { return err("InvalidHandle", "null engine") };
    let g = &e.0.graph;
    let m = e.0.metric();
    to_c(json!({
        "code": "Ok",
        "profile": g.meta.profile,
        "segments": g.meta.num_segments,
        "bbox": g.meta.bbox,
        "metric_version": m.version,
        "customize_ms": m.customize_ms,
        "avoid_areas": e.0.area_names(),
    }))
}

/// Routes through `n` points given as `lonlat` = [lon0, lat0, lon1, lat1, ...], with
/// steps and a polyline6 geometry. `exclude` (optional) names an area to avoid.
///
/// # Safety
/// `e` must be a live handle; `lonlat` must point to `2 * n` doubles.
#[no_mangle]
pub unsafe extern "C" fn shepherd_route(
    e: *const ShepherdEngine,
    lonlat: *const f64,
    n: usize,
    exclude: *const c_char,
) -> *mut c_char {
    let Some(e) = e.as_ref() else { return err("InvalidHandle", "null engine") };
    if lonlat.is_null() || n < 2 {
        return err("InvalidQuery", "need at least two coordinates");
    }
    let raw = std::slice::from_raw_parts(lonlat, 2 * n);
    let coords: Vec<Coord> = raw.chunks_exact(2).map(|p| Coord::from_deg(p[1], p[0])).collect();
    let opts = RouteOptions {
        steps: true,
        overview: true,
        geometry: GeometryFormat::Polyline6,
        exclude: str_arg(exclude).map(str::to_string),
    };
    match catch_unwind(AssertUnwindSafe(|| e.0.route(&coords, &opts))) {
        Ok(Ok(r)) => match serde_json::to_value(&r) {
            Ok(v) => to_c(v),
            Err(x) => err("InternalError", x),
        },
        Ok(Err(x)) => {
            let code = match x {
                RouteError::NoSegment(_) => "NoSegment",
                RouteError::NoRoute(..) => "NoRoute",
                RouteError::InvalidQuery(_) => "InvalidQuery",
            };
            err(code, x)
        }
        Err(_) => err("InternalError", "engine panicked"),
    }
}

/// Sets how traffic lights are costed: 0 = estimate, 1 = timed-only, 2 = off.
/// Re-customizes the metric (tens of ms). Returns the new metric version.
///
/// # Safety
/// `e` must be a live handle.
#[no_mangle]
pub unsafe extern "C" fn shepherd_set_signal_mode(e: *const ShepherdEngine, mode: u8) -> u64 {
    let Some(e) = e.as_ref() else { return 0 };
    let mode = match mode {
        1 => SignalMode::TimedOnly,
        2 => SignalMode::Off,
        _ => SignalMode::Estimate,
    };
    if e.0.overrides().signal_mode == mode {
        return e.0.metric().version;
    }
    e.0.update(|o| o.signal_mode = mode).0
}

/// Signalised intersections inside [west, south, east, north] (at most 3000).
///
/// # Safety
/// `e` must be a live handle.
#[no_mangle]
pub unsafe extern "C" fn shepherd_lights(e: *const ShepherdEngine, west: f64, south: f64, east: f64, north: f64) -> *mut c_char {
    let Some(e) = e.as_ref() else { return err("InvalidHandle", "null engine") };
    let m = e.0.metric();
    let lights = lights_in(&e.0.graph, &m.signal_waits, [west, south, east, north], 3000);
    to_c(json!({
        "code": "Ok",
        "lights": lights.iter().map(|c| [c.lon(), c.lat()]).collect::<Vec<_>>(),
    }))
}
