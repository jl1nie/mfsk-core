#!/usr/bin/env bash
# Build libmfsk with cargo, then run the Swift tests against it.
#
# Two things this script exists to supply, neither of which belongs in
# `Package.swift`:
#
# 1. **The library search path.** `Package.swift` deliberately carries
#    no `unsafeFlags` — a package that has them cannot be used as a
#    dependency, which for a binding is fatal — so the `-L` for
#    `target/release` is passed here instead. An app consuming this
#    package passes its own, or links a prebuilt static library.
# 2. **A developer dir that has XCTest.** XCTest ships with Xcode, not
#    with the Command Line Tools, so on a machine where `xcode-select`
#    points at the CLT `swift test` fails to resolve the module. If
#    Xcode is installed, point at it for this invocation only rather
#    than asking the user to change a global setting with sudo.
set -euo pipefail

cd "$(dirname "$0")/.."
PACKAGE_DIR="$(pwd)"
REPO_ROOT="$(cd ../.. && pwd)"
LIB_DIR="$REPO_ROOT/target/release"

cargo build --manifest-path "$REPO_ROOT/Cargo.toml" -p mfsk-ffi --release

xctest_in() { [[ -d "$1/Platforms/MacOSX.platform/Developer/Library/Frameworks/XCTest.framework" ]]; }

if [[ -z "${DEVELOPER_DIR:-}" && "$(uname -s)" == "Darwin" ]]; then
    if ! xctest_in "$(xcode-select -p)" && xctest_in /Applications/Xcode.app/Contents/Developer; then
        export DEVELOPER_DIR=/Applications/Xcode.app/Contents/Developer
        echo "note: using $DEVELOPER_DIR for XCTest (the Command Line Tools do not ship it)"
    fi
fi

exec swift test \
    --package-path "$PACKAGE_DIR" \
    -Xlinker -L"$LIB_DIR" \
    -Xlinker -rpath -Xlinker "$LIB_DIR" \
    "$@"
