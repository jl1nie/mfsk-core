#!/usr/bin/env bash
# Golden cases for msg::jt72 (pack_message / unpack): a deterministic list of messages and word
# patterns, each run through WSJT-X's own packmsg / unpackmsg (scripts/jt72/jt72_oracle.f90, built
# from the v3.2.0-rc1 tag by scripts/jt72/build_oracle.sh).
#
#   scripts/jt72/gen_cases.sh [oracle] [out-file]
#
#   oracle    target/jt72-oracle/jt72_oracle
#   out-file  mfsk-core/tests/fixtures/jt72_rc1.tsv
#
# Rows: `P` message -> itype, words, text (packmsg then unpackmsg); `U` words -> text (unpackmsg alone,
# for the patterns packmsg never makes: every prefix/suffix range edge, the NUL-filled `addpfx`,
# the grid-as-location range, text bits). Seeded, so the file reproduces byte for byte.
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
ORACLE="${1:-$REPO_ROOT/target/jt72-oracle/jt72_oracle}"
OUT="${2:-$REPO_ROOT/mfsk-core/tests/fixtures/jt72_rc1.tsv}"
[ -x "$ORACLE" ] || { echo "no $ORACLE - run scripts/jt72/build_oracle.sh first" >&2; exit 1; }
COMMIT="$(cat "$(dirname "$ORACLE")/exported-commit" 2>/dev/null || echo unknown)"
python3 -I "$(dirname "$0")/gen_cases.py" "$ORACLE" "$OUT" "$COMMIT"
