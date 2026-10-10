#!/usr/bin/env bash
# Build scripts/pskreporter/spot_oracle.cpp against WSJT-X's own tokens_re and libwsjt_fort.
#
#   scripts/build_jt9_upstream.sh ../WSJT-X v3.3.0-beta1           # leaves libwsjt_fort_omp.a
#   scripts/pskreporter/build_spot_oracle.sh [WSJT-X-repo] [tag]
#
# Output: target/pskreporter/spot_oracle. Needs Qt5Core's development files and g++.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
WSJTX="${1:-$REPO_ROOT/../WSJT-X}"
TAG="${2:-v3.3.0-beta1}"
B="$REPO_ROOT/target/upstream/build-$TAG"
[ -f "$B/libwsjt_fort_omp.a" ] || { echo "no $B/libwsjt_fort_omp.a: run scripts/build_jt9_upstream.sh first" >&2; exit 1; }
OUT="$REPO_ROOT/target/pskreporter"
mkdir -p "$OUT/spot"
# the regular expression, exactly as upstream has it: from `QRegularExpression tokens_re {R"(` to its closing `};`
git -C "$WSJTX" show "$TAG:Decoder/decodedtext.cpp" | python3 -c '
import sys
t = sys.stdin.read()
a = t.index("QRegularExpression tokens_re {")
b = t.index("QRegularExpression::ExtendedPatternSyntaxOption};", a) + len("QRegularExpression::ExtendedPatternSyntaxOption};")
print("static " + t[a:b])
' > "$OUT/spot/tokens_re.inc"
# Radio::is_standard_callsign, likewise: from its signature to the closing brace of the function
git -C "$WSJTX" show "$TAG:Radio.cpp" | python3 -c '
import sys
t = sys.stdin.read()
a = t.index("bool is_standard_callsign (QString const& w)")
b = t.index("\n  }\n", a) + len("\n  }\n")
print(t[a:b].replace("bool is_standard_callsign", "static bool is_standard_callsign", 1).replace("\n  }\n", "\n}\n"))
' > "$OUT/spot/std_call.inc"
FLAGS=$(pkg-config --cflags --libs Qt5Core)
g++ -std=c++17 -O1 -fPIC -fopenmp -I"$OUT/spot" "$SCRIPT_DIR/spot_oracle.cpp" \
    "$B/libwsjt_fort_omp.a" "$B/libwsjt_cxx.a" -lgfortran -lfftw3f_omp -lfftw3f -lfftw3_omp -lfftw3 -lstdc++ -lm \
    $FLAGS -o "$OUT/spot_oracle"
echo "built from $TAG: $OUT/spot_oracle"
