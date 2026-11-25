#!/bin/bash
# Builds the iPhone app and, with a device, installs it.
#   scripts/build.sh sim               build for the iOS simulator
#   scripts/build.sh device [UDID]     build, sign and install on a connected iPhone
# Signing uses SHEPHERD_TEAM (an Apple developer team ID; a free Personal Team works,
# but its apps expire after 7 days) and SHEPHERD_BUNDLE_ID.
set -euo pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)"
FFI="$HERE/../../routing-engine/ffi"
DATA="$HERE/../../routing-engine/data"
WHAT="${1:-sim}"
export SHEPHERD_TEAM="${SHEPHERD_TEAM:-}"
export SHEPHERD_BUNDLE_ID="${SHEPHERD_BUNDLE_ID:-local.shepherd-rs.ShepherdPhone}"
export IPHONEOS_DEPLOYMENT_TARGET=17.0

for f in il.graph il-foot.graph judea-samaria.geojson; do
    [ -f "$DATA/$f" ] || { echo "missing $DATA/$f (see README: preprocess the graphs first)"; exit 1; }
done

case "$WHAT" in
    sim) TARGET=aarch64-apple-ios-sim; SDK=iphonesimulator; DEST="generic/platform=iOS Simulator" ;;
    device)
        TARGET=aarch64-apple-ios; SDK=iphoneos
        # Build for the connected iPhone, so automatic signing can register it with the team.
        UDID="${2:-$(xcrun devicectl list devices 2>/dev/null | awk '/physical/ && !/unavailable/ {for(i=1;i<=NF;i++) if ($i ~ /^[0-9A-F]{8}-[0-9A-F]{16}$/) print $i}' | head -1)}"
        [ -n "$UDID" ] || { echo "no paired iPhone found (connect it with USB and trust this Mac)"; exit 1; }
        DEST="platform=iOS,id=$UDID" ;;
    *) echo "usage: $0 sim|device [UDID]"; exit 1 ;;
esac

echo "==> building routing engine for $TARGET"
(cd "$FFI" && cargo build --release --quiet --target "$TARGET")

echo "==> generating Xcode project"
(cd "$HERE" && xcodegen generate --quiet)

echo "==> building app ($SDK)"
EXTRA=()
if [ "$WHAT" = device ]; then
    [ -n "$SHEPHERD_TEAM" ] || { echo "set SHEPHERD_TEAM to your Apple developer team ID"; exit 1; }
    EXTRA=(-allowProvisioningUpdates -allowProvisioningDeviceRegistration)
else
    EXTRA=(CODE_SIGNING_ALLOWED=NO)
fi
xcodebuild -project "$HERE/ShepherdPhone.xcodeproj" -scheme Shepherd -configuration Release \
    -sdk "$SDK" -destination "$DEST" -derivedDataPath "$HERE/build/dd" "${EXTRA[@]}" \
    -quiet build
APP="$HERE/build/dd/Build/Products/Release-$SDK/Shepherd.app"
echo "==> built $APP ($(du -sh "$APP" | cut -f1))"

if [ "$WHAT" = device ]; then
    echo "==> installing on $UDID"
    xcrun devicectl device install app --device "$UDID" "$APP"
fi
