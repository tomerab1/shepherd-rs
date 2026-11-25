#!/bin/bash
# Nightly data refresh: download a fresh OSM extract, rebuild both profiles and the
# Judea & Samaria area, swap the files in atomically, and tell running servers to reload
# (SIGHUP). A server keeps serving the old mapping until the new one is ready: no downtime.
#
#   scripts/refresh.sh [data-dir] [extract-url]
set -euo pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)"
DATA="${1:-$HERE/data}"
URL="${2:-https://download.geofabrik.de/asia/israel-and-palestine-latest.osm.pbf}"
BIN="$HERE/target/release/shepherd"
WORK="$(mktemp -d "$DATA/.refresh.XXXXXX")"
trap 'rm -rf "$WORK"' EXIT

[ -x "$BIN" ] || (cd "$HERE" && cargo build --release --quiet)
echo "downloading $URL"
curl -fsSL -o "$WORK/extract.osm.pbf" "$URL"
"$BIN" preprocess "$WORK/extract.osm.pbf" "$WORK/il.graph" --profile driving
"$BIN" preprocess "$WORK/extract.osm.pbf" "$WORK/il-foot.graph" --profile walking
"$BIN" extract-area "$WORK/extract.osm.pbf" --relation 1803010 --name judea-samaria --out "$WORK/judea-samaria.geojson"
# Sanity check before swapping anything in.
"$BIN" verify "$WORK/il.graph" --sources 5 --targets 100

for f in il.graph il-foot.graph judea-samaria.geojson; do
    mv -f "$WORK/$f" "$DATA/$f"   # rename = atomic; mapped old files stay valid
done
mv -f "$WORK/extract.osm.pbf" "$DATA/israel-and-palestine-latest.osm.pbf"
pkill -HUP -f "shepherd serve" && echo "servers reloaded" || echo "no running server"
