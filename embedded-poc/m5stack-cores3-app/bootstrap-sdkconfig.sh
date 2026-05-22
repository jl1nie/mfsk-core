#!/usr/bin/env bash
# Generate sdkconfig.gen.defaults with this checkout's absolute partition
# CSV path. Required once on a fresh clone (and after moving the repo to
# a different path). See sibling `m5stack-s3-app/bootstrap-sdkconfig.sh`
# for rationale.
set -euo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)"
cat > "$DIR/sdkconfig.gen.defaults" <<EOF
CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="$DIR/partitions.csv"
EOF
echo "wrote $DIR/sdkconfig.gen.defaults"
