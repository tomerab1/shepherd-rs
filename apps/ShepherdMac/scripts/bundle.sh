#!/bin/bash
# Builds Shepherd.app: the SwiftUI client plus the bundled Rust routing engine and graph.
#   scripts/bundle.sh [install-dir]
# Routing data (graphs, avoid areas) is installed to ~/Library/Application Support/Shepherd.
set -euo pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)"
ENGINE="$HERE/../../routing-engine"
INSTALL_DIR="${1:-}"
DATA_DIR="$HOME/Library/Application Support/Shepherd"
OUT="$HERE/build/Shepherd.app"

echo "==> building routing engine"
(cd "$ENGINE" && cargo build --release --quiet)
echo "==> building app"
(cd "$HERE" && swift build -c release --quiet)

echo "==> assembling $OUT"
rm -rf "$OUT"
mkdir -p "$OUT/Contents/MacOS" "$OUT/Contents/Resources"
cp "$HERE/.build/release/Shepherd" "$OUT/Contents/MacOS/Shepherd"
cp "$ENGINE/target/release/shepherd" "$OUT/Contents/Resources/shepherd"

ICONSET="$HERE/build/AppIcon.iconset"
rm -rf "$ICONSET"
swift "$HERE/scripts/make_icon.swift" "$ICONSET"
iconutil -c icns "$ICONSET" -o "$OUT/Contents/Resources/AppIcon.icns"

cat > "$OUT/Contents/Info.plist" <<PLIST
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <key>CFBundleName</key><string>Shepherd</string>
    <key>CFBundleDisplayName</key><string>Shepherd</string>
    <key>CFBundleIdentifier</key><string>local.shepherd-rs.Shepherd</string>
    <key>CFBundleExecutable</key><string>Shepherd</string>
    <key>CFBundleIconFile</key><string>AppIcon</string>
    <key>CFBundlePackageType</key><string>APPL</string>
    <key>CFBundleShortVersionString</key><string>1.0</string>
    <key>CFBundleVersion</key><string>1</string>
    <key>LSMinimumSystemVersion</key><string>14.0</string>
    <key>LSApplicationCategoryType</key><string>public.app-category.navigation</string>
    <key>NSHighResolutionCapable</key><true/>
    <key>NSLocationUsageDescription</key><string>Shepherd uses your location to start routes from where you are and to guide you turn by turn.</string>
    <key>NSLocationWhenInUseUsageDescription</key><string>Shepherd uses your location to start routes from where you are and to guide you turn by turn.</string>
    <key>NSAppTransportSecurity</key>
    <dict><key>NSAllowsLocalNetworking</key><true/></dict>
</dict>
</plist>
PLIST

codesign --force -s - "$OUT/Contents/Resources/shepherd"
codesign --force -s - "$OUT"
echo "==> built $(du -sh "$OUT" | cut -f1) app"

echo "==> installing routing data to $DATA_DIR"
mkdir -p "$DATA_DIR"
for f in il.graph il-foot.graph judea-samaria.geojson; do
    if [ -f "$ENGINE/data/$f" ]; then
        # Copy then rename, so a running engine keeps its (still valid) old mapping.
        cp "$ENGINE/data/$f" "$DATA_DIR/.$f.tmp" && mv -f "$DATA_DIR/.$f.tmp" "$DATA_DIR/$f"
    fi
done

if [ -n "$INSTALL_DIR" ]; then
    mkdir -p "$INSTALL_DIR"
    rm -rf "$INSTALL_DIR/Shepherd.app"
    ditto "$OUT" "$INSTALL_DIR/Shepherd.app"
    echo "==> installed to $INSTALL_DIR/Shepherd.app"
fi
