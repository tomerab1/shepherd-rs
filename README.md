# Shepherd

A road routing engine in Rust (driving and walking) with live traffic, road closures,
traffic-light timing and avoidable areas, plus a native macOS app with turn-by-turn navigation.

```
routing-engine/     Rust engine + axum HTTP server (`shepherd` binary)
apps/ShepherdMac/   SwiftUI + MapKit app that bundles and runs the engine
```

## How it works

| Stage | What happens | Israel (driving) |
|---|---|---|
| Import | Two parallel PBF passes: routable ways + turn restrictions, then only the nodes they use | 1.1 s |
| Network | Ways split at junctions into segments; shape points kept as geometry only | 606k segments |
| Edge-based graph (driving) | One node per *direction of a segment*, one arc per legal turn. Via-node restrictions (`no_*`, `only_*`) remove arcs; via-way restrictions duplicate the via chain for the restricted entry; turn costs live on arcs | 1.0M nodes, 12.7k of 12.8k restrictions |
| Node-based graph (walking) | Junctions as nodes, directed segments as arcs — exact when there are no turn rules, and far smaller | 800k nodes, 3.4M CCH edges |
| Order | Nested dissection with inertial-flow separators (unit-capacity max-flow), halves in parallel | 4 s |
| CCH topology | Chordal supergraph + elimination tree. Metric-independent | 6.4M edges, depth 431 |
| Customization | Lower-triangle relaxation, level by level, each level in parallel. **Only this runs again when traffic changes** | ~0.2–0.3 s |
| Query | Walk up the elimination tree from both ends; no priority queue. Scratch state is per-ancestor (KBs), not per-node | ~140 µs + unpacking |

**Why CCH:** a plain Contraction Hierarchy bakes travel times into its shortcuts, so any traffic
change means re-preprocessing. With CCH, the slow part (the order) ignores weights, and new weights
are applied in a sub-second parallel *customization*. The result is published atomically
(`ArcSwap`); requests already in flight finish on the metric they started with.

**Memory:** the graph file is a set of 64-byte-aligned plain-old-data arrays that the server
`mmap`s and uses in place (zero-copy `Arr<T>`): startup takes milliseconds, the OS pages data in
on demand, and several processes share one copy. Only the metric (edge weights) lives on the heap.
Arc weights are computed on the fly (`node_w[head] + entry + turn`) instead of stored, and
avoid-area metrics are customized lazily on first use. The walking profile routes on a
node-based graph (6.6× fewer CCH edges than edge-based). Serving both profiles takes
**~123 MB** of physical memory (~157 MB once avoid-area metrics are in use).

| Measured on an 8-core M-series Mac (Israel) | Driving | Walking |
|---|---|---|
| Preprocessing | 7.7 s | 6.7 s |
| Query + unpack (1 thread) | ~140 µs | ~185 µs |
| Full route incl. steps/geometry, all cores | ~9.7k routes/s | ~9.4k routes/s |
| Re-customization after a traffic update | ~0.2 s | ~0.09 s |
| Server start (mmap + customize) | 1.5 s for both | |

**Profiles:** `driving` (one-ways, turn restrictions, turn costs, left turns cost more) and
`walking` (footpaths/steps/pedestrian streets, no one-way or turn rules, no motorways/trunks unless
there is a sidewalk; node-based graph). Each profile is its own graph file.

**Traffic lights:** OSM maps a signalised intersection with several light nodes, one per approach,
carriageway or crossing, and often only on some of its legs. So waits are per **intersection**,
not per light node:
- Lights within 40 m of each other form one intersection.
- Every junction within 25 m of one of its lights belongs to it. So do junctions linked to those by
  internal segments under 30 m (dual carriageways, slip lanes).
- Driving *through* any junction of a signalised intersection costs one wait. Lights far from any
  junction are mid-block crossings.

On 8 test routes, every signalised intersection driven through is counted (`examples/route_lights`
checks this). Pedestrian `crossing=traffic_signals` lights count for walking. Waits are applied by
the metric, not baked in:
- `signal_mode=estimate` (default): lights without data get a typical wait (12 s per intersection
  for cars, Webster's `C·(1−g/C)²/2` with C = 90 s and g/C = 0.45).
- `signal_mode=timed-only`: only intersections you uploaded timing for count; the rest are ignored.
- `signal_mode=off`: lights are ignored entirely.

Routes list the intersections with lights they pass.

**Live traffic (HERE):** with `HERE_API_KEY` set, the server polls HERE Traffic v7 `/flow` for the
graph's bounding box (1°×1° tiles; Israel is 8 requests per poll, every 15 min by default, which
fits the free plan). Each flow item's shape is map-matched onto our segments, with direction
checked against the segment bearing. HERE's free-flow/current speed ratio becomes a travel-time
factor, and closed roads are closed. Each poll replaces the previous feed and re-customizes
(~0.2 s). Status is at `GET /admin/traffic-feed`; `POST /admin/traffic-feed` polls now.

**Avoid areas:** any polygon (GeoJSON) can be loaded as an avoidable area. A route with
`exclude=<name>` uses a metric where travel inside the area weighs 40× more. Routes stay out
whenever an alternative exists, but trips that start or end inside still work, and reported
durations are real times. `judea-samaria` is extracted from OSM relation 1803010.

## Build and run

```bash
cd routing-engine
cargo build --release

# Data: full extract (with footpaths and turn restrictions) from Geofabrik
curl -L -o data/israel-and-palestine-latest.osm.pbf https://download.geofabrik.de/asia/israel-and-palestine-latest.osm.pbf
./target/release/shepherd preprocess data/israel-and-palestine-latest.osm.pbf data/il.graph --profile driving
./target/release/shepherd preprocess data/israel-and-palestine-latest.osm.pbf data/il-foot.graph --profile walking
./target/release/shepherd extract-area data/israel-and-palestine-latest.osm.pbf --relation 1803010 --name judea-samaria --out data/judea-samaria.geojson

./target/release/shepherd serve data/il.graph data/il-foot.graph \
    --avoid-area judea-samaria=data/judea-samaria.geojson --bind 127.0.0.1:5000
```

Other commands: `route` (one route as JSON), `bench`, `verify` (CCH vs. Dijkstra on random pairs),
`extract-area --find <name>` (list boundary relations). `--threads N` limits worker threads.

## HTTP API

OSRM-compatible where it overlaps, so OSRM clients work.

| Endpoint | |
|---|---|
| `GET /route/v1/{driving\|foot}/{lon,lat;lon,lat;...}` | `steps=true`, `overview=full\|false`, `geometries=polyline\|polyline6\|geojson`, `exclude=judea-samaria`. Up to 25 waypoints. Response adds `traffic_lights` and `metric_version` |
| `GET /nearest/v1/{profile}/{lon,lat}` | Snapped position, road name, OSM way id |
| `GET /lights/v1/{profile}?bbox=w,s,e,n` | Traffic lights in a box (≤ 0.5°), one per intersection |
| `GET /health` | Loaded profiles, graph stats, metric versions |
| `GET /metrics` | Prometheus: request counts, latency histogram, metric versions |
| `POST /admin/updates?profile=driving` | Live traffic, closures, signal timing, `signal_mode` (below) |
| `GET` / `POST /admin/traffic-feed` | HERE feed status / poll now |
| `GET` / `DELETE /admin/updates?profile=...` | List / clear overrides |

Admin endpoints require `x-admin-token` when `SHEPHERD_ADMIN_TOKEN` is set.

```json
{
  "traffic":  [{ "way_id": 123, "speed_kmh": 20, "direction": "forward" },
               { "near": [34.79, 32.07], "radius_m": 80, "factor": 3.0 }],
  "closures": [{ "near": [34.79, 32.07], "radius_m": 40 }],
  "reopen":   [{ "way_id": 456 }],
  "signals":  [{ "osm_id": 1439390172, "cycle_s": 120, "green_s": 35 },
               { "near": [34.78, 32.08], "delay_s": 25 }],
  "signal_mode": "timed-only",
  "reset": false
}
```

Each update re-customizes and swaps in a new metric (~0.2 s for driving).

## macOS app

```bash
apps/ShepherdMac/scripts/bundle.sh /Applications   # builds engine + app, installs data to ~/Library/Application Support/Shepherd
```

The app starts the bundled engine on `127.0.0.1:5057` and stops it on quit. Features:
- Drive / Walk modes, "Avoid Judea & Samaria"
- Start and destination by **address or place name** (Apple MapKit autocomplete, Hebrew or
  English), `lat, lon`, clicking the map, or **Use my location**
- Traffic settings: traffic-light mode, all lights in view when zoomed in, and a HERE API key
  (kept in the Keychain) for live traffic
- Turn-by-turn steps, traffic lights on the map
- **Navigation**:
  - Next maneuver and distance to it
  - Remaining distance; time left at your measured pace (walking) or traffic-aware (driving)
  - Arrival time and current speed
  - Re-routes automatically when you leave the route
  - **Simulate** plays the route at 1×, 5× or 20× (switchable during navigation)
- Live incidents: click a road to close it or add a jam, and see the route change

## Operations

- `routing-engine/scripts/refresh.sh` downloads a fresh extract and rebuilds everything. It
  verifies the result, swaps the files atomically, and sends `SIGHUP` to running servers, which
  reload with no downtime.
- `routing-engine/Dockerfile`: multi-stage build that runs as non-root.
- Logs go to stderr (`tracing`). The app writes the engine log to `~/Library/Logs/Shepherd/engine.log`.

## Tests

`cargo test --release` covers:
- Unit tests: profiles, polyline encoding, ordering quality, point-in-polygon.
- Synthetic networks: `no_`/`only_` and via-way restrictions, one-ways, closures and traffic, a
  target behind the start on a one-way (needs a loop), traffic-light timing, step generation,
  and the node-based walking graph.
- A randomized CCH-vs-Dijkstra check with random restrictions.
- Israel regressions (run when `data/il.graph` exists).

## Possible next steps

- A TomTom provider for the traffic feed (HERE is implemented).
- An iOS build of the app (the engine would run on a server or on the Mac over the LAN).
