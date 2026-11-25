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

**Traffic lights:** OSM `highway=traffic_signals` nodes are kept with the graph. Their delay is
applied by the metric, not baked in. The default is the expected wait for a typical signal (12 s at
junctions for cars, from Webster's uniform delay `C·(1−g/C)²/2` with C = 90 s, g/C = 0.45). Real
timing data overrides it per light at runtime. Routes list the lights they pass.

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
| `GET /health` | Loaded profiles, graph stats, metric versions |
| `GET /metrics` | Prometheus: request counts, latency histogram, metric versions |
| `POST /admin/updates?profile=driving` | Live traffic, closures, signal timing (below) |
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
- Start and destination by typing `lat, lon` or clicking the map, or **Use my location**
- Turn-by-turn steps, traffic lights on the map
- **Navigation**:
  - Next maneuver and distance to it
  - Remaining distance; time left at your measured pace (walking) or traffic-aware (driving)
  - Arrival time and current speed
  - Re-routes automatically when you leave the route
  - **Simulate** moves along the route to try it at a desk
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

- **Traffic feed ingestion.** The admin API takes speeds by way or location. A poller for HERE
  Traffic v7 flow (by bounding box) or TomTom would map their segments onto ours (needs an API key).
- An iOS build of the app (the engine would run on a server or on the Mac over the LAN).
