#!/usr/bin/env bash
# Generate sdkconfig.gen.defaults with this checkout's absolute partition
# CSV path. Required once on a fresh clone (and after moving the repo to
# a different path), because esp-idf-sys's build script runs before our
# build.rs and would otherwise hit a missing-file error.
#
# build.rs re-generates the same file on every subsequent build, so this
# script only matters for the first invocation.
set -euo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)"
cat > "$DIR/sdkconfig.gen.defaults" <<EOF
CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="$DIR/partitions.csv"
EOF
echo "wrote $DIR/sdkconfig.gen.defaults"
