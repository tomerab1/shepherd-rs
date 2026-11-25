//! Live traffic from the HERE Traffic API v7 (`/v7/flow`).
//!
//! Every poll fetches flow for the graph's bounding box in 1°×1° tiles, map-matches each
//! flow item's shape onto our road segments (nearest segment whose direction agrees with the
//! flow direction), and turns HERE's free-flow/current speed ratio into a travel-time factor
//! per directed segment. Using the *ratio* keeps the feed independent of how HERE's absolute
//! speeds compare to our profile speeds. Closed roads (`traversability: closed`, or a jam
//! factor of 10) are closed. The result replaces the previous feed and re-customizes.

use crate::geo::{bearing, haversine, turn_angle, Coord};
use crate::graph::Graph;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::time::Duration;
#[cfg(feature = "server")]
use std::time::Instant;
#[cfg(feature = "server")]
use crate::engine::Engine;

#[derive(Clone, Debug)]
pub struct FeedConfig {
    pub api_key: String,
    /// Base URL; overridable for tests or a proxy.
    pub base_url: String,
    pub interval: Duration,
}

impl FeedConfig {
    pub const DEFAULT_URL: &'static str = "https://data.traffic.hereapi.com";
}

/// Status of the most recent poll, exposed at `GET /admin/traffic-feed`.
#[derive(Clone, Debug, Default, Serialize)]
pub struct FeedStatus {
    pub enabled: bool,
    pub polls: u64,
    pub last_poll_unix: u64,
    pub ok: bool,
    pub error: Option<String>,
    pub requests: usize,
    pub flow_items: usize,
    pub matched_segments: usize,
    pub slowed_segments: usize,
    pub closed_segments: usize,
    pub fetch_ms: u64,
    pub customize_ms: f64,
    pub metric_version: u64,
}

#[derive(Deserialize, Debug)]
pub struct FlowResponse {
    #[serde(default)]
    pub results: Vec<FlowItem>,
}

#[derive(Deserialize, Debug)]
pub struct FlowItem {
    pub location: FlowLocation,
    #[serde(rename = "currentFlow")]
    pub current: CurrentFlow,
}

#[derive(Deserialize, Debug)]
pub struct FlowLocation {
    pub shape: Option<Shape>,
}

#[derive(Deserialize, Debug)]
pub struct Shape {
    #[serde(default)]
    pub links: Vec<Link>,
}

#[derive(Deserialize, Debug)]
pub struct Link {
    #[serde(default)]
    pub points: Vec<Point>,
}

#[derive(Deserialize, Debug, Clone, Copy)]
pub struct Point {
    pub lat: f64,
    pub lng: f64,
}

#[derive(Deserialize, Debug)]
pub struct CurrentFlow {
    /// m/s (may be capped at the speed limit).
    pub speed: Option<f64>,
    /// m/s
    #[serde(rename = "freeFlow")]
    pub free_flow: Option<f64>,
    #[serde(rename = "jamFactor")]
    pub jam_factor: Option<f64>,
    pub traversability: Option<String>,
}

/// Bounding boxes of at most 1°×1° covering `bbox` ([west, south, east, north]).
pub fn tiles(bbox: [f64; 4]) -> Vec<[f64; 4]> {
    let mut out = Vec::new();
    let mut s = bbox[1];
    while s < bbox[3] {
        let n = (s + 1.0).min(bbox[3]);
        let mut w = bbox[0];
        while w < bbox[2] {
            let e = (w + 1.0).min(bbox[2]);
            out.push([w, s, e, n]);
            w = e;
        }
        s = n;
    }
    out
}

/// Result of matching a set of flow items onto the graph.
#[derive(Default, Debug)]
pub struct Matched {
    pub factors: BTreeMap<u32, [Option<f32>; 2]>,
    pub matched_segments: usize,
    pub slowed: usize,
    pub closed: usize,
}

/// Map-matches flow items onto directed segments.
pub fn match_flow(g: &Graph, items: &[FlowItem]) -> Matched {
    // Per directed segment: the strongest slowdown reported (closure = 0.0).
    let mut factors: BTreeMap<u32, [Option<f32>; 2]> = BTreeMap::new();
    let mut touched = 0usize;
    for item in items {
        let c = &item.current;
        let closed = c.traversability.as_deref() == Some("closed") || c.jam_factor.unwrap_or(0.0) >= 10.0;
        let factor: f32 = if closed {
            0.0
        } else {
            match (c.free_flow, c.speed) {
                (Some(f), Some(v)) if f > 0.0 && v > 0.0 => (f / v).clamp(1.0, 10.0) as f32,
                _ => continue,
            }
        };
        let Some(shape) = &item.location.shape else { continue };
        for link in &shape.links {
            for w in link.points.windows(2) {
                let (a, b) = (Coord::from_deg(w[0].lat, w[0].lng), Coord::from_deg(w[1].lat, w[1].lng));
                let len = haversine(a, b);
                if len < 1.0 {
                    continue;
                }
                let dir = bearing(a, b);
                // Sample every ~15 m along the shape piece.
                let n = (len / 15.0).ceil().max(1.0) as usize;
                for i in 0..n {
                    let t = (i as f64 + 0.5) / n as f64;
                    let p = crate::geo::lerp(a, b, t);
                    let Some(s) = g.snap(p, 12.0) else { continue };
                    let diff = turn_angle(s.bearing, dir).abs();
                    let k = if diff < 45.0 {
                        0
                    } else if diff > 135.0 {
                        1
                    } else {
                        continue; // crossing road, not this one
                    };
                    let entry = factors.entry(s.segment).or_default();
                    if entry[k].is_none() {
                        touched += 1;
                    }
                    entry[k] = Some(match entry[k] {
                        // Closure wins; otherwise keep the strongest slowdown.
                        Some(prev) if prev == 0.0 || factor == 0.0 => 0.0,
                        Some(prev) => prev.max(factor),
                        None => factor,
                    });
                }
            }
        }
    }
    // Drop insignificant slowdowns to keep the override set small.
    let mut out = Matched {
        matched_segments: touched,
        ..Default::default()
    };
    for (seg, f) in factors {
        let keep = |x: Option<f32>| x.filter(|&v| v == 0.0 || v >= 1.15);
        let kept = [keep(f[0]), keep(f[1])];
        if kept.iter().any(|x| x.is_some()) {
            for v in kept.iter().flatten() {
                if *v == 0.0 {
                    out.closed += 1;
                } else {
                    out.slowed += 1;
                }
            }
            out.factors.insert(seg, kept);
        }
    }
    out
}

/// Fetches one tile.
#[cfg(feature = "server")]
pub async fn fetch_tile(client: &reqwest::Client, cfg: &FeedConfig, t: [f64; 4]) -> anyhow::Result<FlowResponse> {
    let url = format!("{}/v7/flow", cfg.base_url.trim_end_matches('/'));
    let bbox = format!("bbox:{:.5},{:.5},{:.5},{:.5}", t[0], t[1], t[2], t[3]);
    let resp = client
        .get(url)
        .query(&[("in", bbox.as_str()), ("locationReferencing", "shape"), ("apiKey", cfg.api_key.as_str())])
        .send()
        .await?;
    let status = resp.status();
    if !status.is_success() {
        let body = resp.text().await.unwrap_or_default();
        anyhow::bail!("HERE returned {status}: {}", body.chars().take(300).collect::<String>());
    }
    Ok(resp.json().await?)
}

/// One poll: fetch every tile, match, publish. Returns the new status.
#[cfg(feature = "server")]
pub async fn poll_once(client: &reqwest::Client, cfg: &FeedConfig, engine: &std::sync::Arc<Engine>) -> FeedStatus {
    let mut st = FeedStatus {
        enabled: true,
        last_poll_unix: std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_or(0, |d| d.as_secs()),
        ..Default::default()
    };
    let t = Instant::now();
    let mut items = Vec::new();
    for tile in tiles(engine.graph.meta.bbox) {
        st.requests += 1;
        match fetch_tile(client, cfg, tile).await {
            Ok(r) => items.extend(r.results),
            Err(e) => {
                st.error = Some(format!("{e:#}"));
                st.fetch_ms = t.elapsed().as_millis() as u64;
                return st;
            }
        }
    }
    st.fetch_ms = t.elapsed().as_millis() as u64;
    st.flow_items = items.len();
    let engine2 = engine.clone();
    let res = tokio::task::spawn_blocking(move || {
        let m = match_flow(&engine2.graph, &items);
        let counts = (m.matched_segments, m.slowed, m.closed);
        let factors = m.factors;
        let (version, ms) = engine2.update(|o| o.feed_factor = factors);
        (counts, version, ms)
    })
    .await;
    match res {
        Ok(((matched, slowed, closed), version, ms)) => {
            st.ok = true;
            st.matched_segments = matched;
            st.slowed_segments = slowed;
            st.closed_segments = closed;
            st.metric_version = version;
            st.customize_ms = ms;
        }
        Err(e) => st.error = Some(e.to_string()),
    }
    st
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tiles_cover_bbox_in_one_degree_steps() {
        let t = tiles([34.2, 29.4, 35.9, 33.4]);
        assert_eq!(t.len(), 2 * 4);
        assert!(t.iter().all(|b| b[2] - b[0] <= 1.0 + 1e-9 && b[3] - b[1] <= 1.0 + 1e-9));
        assert_eq!(t.last().unwrap()[2], 35.9);
    }

    #[test]
    fn parses_here_v7_flow_json() {
        let json = r#"{"sourceUpdated":"2026-11-25T12:00:00Z","results":[{"location":{"description":"X","length":120.0,
          "shape":{"links":[{"points":[{"lat":32.0,"lng":34.8},{"lat":32.001,"lng":34.8}],"length":111.0,"functionalClass":2}]}},
          "currentFlow":{"speed":5.0,"speedUncapped":5.0,"freeFlow":15.0,"jamFactor":6.5,"confidence":0.9,"traversability":"open"}}]}"#;
        let r: FlowResponse = serde_json::from_str(json).unwrap();
        assert_eq!(r.results.len(), 1);
        assert_eq!(r.results[0].current.free_flow, Some(15.0));
        assert_eq!(r.results[0].location.shape.as_ref().unwrap().links[0].points.len(), 2);
    }
}
