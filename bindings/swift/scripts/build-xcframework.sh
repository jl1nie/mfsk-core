#!/usr/bin/env bash
# Build libmfsk for iOS (device + Apple-silicon simulator) and package
# the two static libraries as one XCFramework.
#
#   bindings/swift/scripts/build-xcframework.sh [output-dir]
#
# Output: <output-dir>/Mfsk.xcframework, default `target/xcframework/`.
# Each slice carries `libmfsk.a`, `mfsk.h` and a module map naming the
# module `CMfsk` — the same name `Sources/CMfsk` gives it — so Swift
# code written against this package imports it unchanged.
#
# Three things worth knowing:
#
# 1. **`mobile` features.** Both slices are built with
#    `--no-default-features --features mobile`, the same set CI's iOS
#    cross-build uses: no rayon, so no lazily spawned pool of
#    `num_cpus` threads outside GCD's QoS (see `mfsk-ffi/Cargo.toml`).
# 2. **The module map has no `link "mfsk"`.** `Sources/CMfsk` needs it
#    because there the library is found by `-L`; an XCFramework slice
#    is linked by Xcode itself, and a `link` line would ask the linker
#    for `-lmfsk` a second time on a search path that does not exist.
# 3. **Xcode, not the Command Line Tools.** The iOS SDKs ship with
#    Xcode. As in `test.sh`, `DEVELOPER_DIR` is pointed at Xcode for
#    this invocation only when `xcode-select` is on the CLT.
#
# After packaging, a two-line Swift program that calls
# `mfsk_abi_version()` is compiled and linked against each slice. That
# catches a broken module map or a missing symbol here instead of in an
# app's build; it does not run anything, since that needs a simulator
# or a device.
set -euo pipefail

# Resolve a caller-supplied output dir before the `cd` below changes
# what a relative path means.
if [[ -n "${1:-}" ]]; then
    mkdir -p "$1"
    OUT_DIR="$(cd "$1" && pwd)"
fi

cd "$(dirname "$0")/.."
REPO_ROOT="$(cd ../.. && pwd)"
OUT_DIR="${OUT_DIR:-$REPO_ROOT/target/xcframework}"
XCFRAMEWORK="$OUT_DIR/Mfsk.xcframework"

# slice id : rust target : swiftc target : SDK
SLICES=(
    "ios-arm64:aarch64-apple-ios:arm64-apple-ios14.0:iphoneos"
    "ios-arm64-simulator:aarch64-apple-ios-sim:arm64-apple-ios14.0-simulator:iphonesimulator"
)

if [[ "$(uname -s)" != "Darwin" ]]; then
    echo "error: an XCFramework needs Xcode, which needs macOS" >&2
    exit 1
fi

ios_sdk_in() { [[ -d "$1/Platforms/iPhoneOS.platform" ]]; }

if [[ -z "${DEVELOPER_DIR:-}" ]] && ! ios_sdk_in "$(xcode-select -p)"; then
    if ios_sdk_in /Applications/Xcode.app/Contents/Developer; then
        export DEVELOPER_DIR=/Applications/Xcode.app/Contents/Developer
        echo "note: using $DEVELOPER_DIR for the iOS SDKs (the Command Line Tools do not ship them)"
    else
        echo "error: no Xcode with the iOS SDK found; install Xcode or set DEVELOPER_DIR" >&2
        exit 1
    fi
fi

installed_targets="$(rustup target list --installed)"
for slice in "${SLICES[@]}"; do
    IFS=: read -r _ rust_target _ _ <<<"$slice"
    if ! grep -qx "$rust_target" <<<"$installed_targets"; then
        echo "error: rust target $rust_target is not installed; run: rustup target add $rust_target" >&2
        exit 1
    fi
done

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

mkdir -p "$WORK/headers"
cp "$REPO_ROOT/mfsk-ffi/include/mfsk.h" "$WORK/headers/"
cat >"$WORK/headers/module.modulemap" <<'EOF'
module CMfsk {
    header "mfsk.h"
    export *
}
EOF

create_args=()
for slice in "${SLICES[@]}"; do
    IFS=: read -r _ rust_target _ _ <<<"$slice"
    cargo build --manifest-path "$REPO_ROOT/Cargo.toml" -p mfsk-ffi --release \
        --target "$rust_target" --no-default-features --features mobile
    create_args+=(-library "$REPO_ROOT/target/$rust_target/release/libmfsk.a" -headers "$WORK/headers")
done

rm -rf "$XCFRAMEWORK"
mkdir -p "$OUT_DIR"
xcodebuild -create-xcframework "${create_args[@]}" -output "$XCFRAMEWORK"

cat >"$WORK/main.swift" <<'EOF'
import CMfsk
print(mfsk_abi_version(), mfsk_version())
EOF
for slice in "${SLICES[@]}"; do
    IFS=: read -r id _ swift_target sdk <<<"$slice"
    xcrun -sdk "$sdk" swiftc -target "$swift_target" \
        -I "$XCFRAMEWORK/$id/Headers" -L "$XCFRAMEWORK/$id" -lmfsk \
        "$WORK/main.swift" -o "$WORK/smoke-$id"
    echo "link check ok: $id"
done

echo "wrote $XCFRAMEWORK"
