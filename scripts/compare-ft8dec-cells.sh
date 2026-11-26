#!/usr/bin/env bash
# Instruction counts of the cells flow (decode_block_cells[_sic]) against ft8dec (icap-sdk apps/ft8, MIT) on the three vendored
# recordings, 8 kHz, same audio for both. macOS: `/usr/bin/time -l` ("instructions retired", as FT8_WINDOW_AND_COARSE_DESIGN.md);
# Linux: valgrind --tool=callgrind. Instruction counts depend on the ISA: compare ratios measured on one machine, not numbers across machines.
# Usage: scripts/compare-ft8dec-cells.sh [workdir]   (default: a fresh mktemp dir; icap-sdk is cloned into it, read-only use)
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
W="${1:-$(mktemp -d)}"; mkdir -p "$W"; echo "workdir $W"
SDK_URL=https://github.com/jl1nie/icap-sdk.git; SDK_REV=1ca9f04
[ -d "$W/icap-sdk" ] || git clone -q "$SDK_URL" "$W/icap-sdk"
git -C "$W/icap-sdk" checkout -q "$SDK_REV"
A="$W/icap-sdk/apps/ft8"; C="$W/icap-sdk/apps/digi-common"; SDK="$W/icap-sdk"

# 1. the 8 kHz audio both sides read, and the cells-side test binary
export CELLS_DUMP_DIR="$W"
BIN=$(cd "$ROOT" && cargo test -p mfsk-core --release --features full,internal-testing,fixed-point --test cells_flow_host_reference --no-run --message-format=json 2>/dev/null \
  | python3 -c 'import sys,json
for l in sys.stdin:
    try: m=json.loads(l)
    except Exception: continue
    if m.get("executable") and m.get("target",{}).get("name")=="cells_flow_host_reference": print(m["executable"])' | tail -1)
[ -x "$BIN" ] || { echo "no test binary"; exit 1; }
"$BIN" cells_flow_matches_ft8dec_on_the_vendored_recordings >/dev/null     # writes <wav>_8k.raw into CELLS_DUMP_DIR

# 2. the ft8dec driver (reads the raw 8 kHz s16, scales by 1/32768 as digiapp.c's src_read does)
cat > "$W/drv.c" <<'EOF'
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "ft8dec.h"
typedef struct { int llr, max_cand, min_score, ldpc_iters, osd, osd_nt, nhmax, passes, nsync_min; float scale, dmax, snr_off; int pct, skip, osd_min, near, interp; } ft8_tune_t;
extern ft8_tune_t ft8_tune;
static int16_t *g; static uint32_t glen;
static int rd(void *c, uint32_t s, float *o, uint32_t n) { (void)c; for (uint32_t i = 0; i < n; i++) o[i] = s + i < glen ? (float)g[s + i] * (1.0f / 32768.0f) : 0.f; return (int)n; }
int main(int argc, char **argv) {
    FILE *f = fopen(argv[1], "rb"); if (!f) return 2;
    fseek(f, 0, SEEK_END); long sz = ftell(f); fseek(f, 0, SEEK_SET);
    g = malloc(sz); if (fread(g, 1, sz, f) != (size_t)sz) return 2; fclose(f); glen = sz / 2;
    if (argc > 4) { ft8_tune.max_cand = atoi(argv[2]); ft8_tune.osd = atoi(argv[3]); ft8_tune.passes = atoi(argv[4]); }
    if (getenv("NODEC")) return 0;
    dsrc_t src = { 0, rd, glen, 8000 };
    static ft8_result_t out[64]; static ft8_state_t st; uint32_t wl = 512 * 1024; void *work = aligned_alloc(8, wl);
    int n = ft8_decode(&src, 200.f, 3000.f, 0.5f, out, 64, work, wl, 0, &st);
    printf("%d\n", n); return 0;
}
EOF
CC=${CC:-cc}; O="-O2 -fno-math-errno -w -I$A -I$A/ft8 -I$C -I$SDK"
OBJ=""
for f in constants crc message text encode; do $CC $O -include "$A/ft8/ft8_port.h" -c "$A/ft8/ft8/$f.c" -o "$W/f_$f.o"; OBJ="$OBJ $W/f_$f.o"; done
$CC $O -include "$A/ft8/ft8_port.h" -c "$A/ft8/ft8_port.c" -o "$W/f_port.o"
$CC $O -c "$C/digi_util.c" -o "$W/digi_util.o"; $CC $O -c "$A/ft8dec.c" -o "$W/ft8dec.o"; $CC $O -c "$W/drv.c" -o "$W/drv.o"
$CC "$W/drv.o" "$W/ft8dec.o" "$W/digi_util.o" $OBJ "$W/f_port.o" -lm -o "$W/ft8dec_drv"

# 3. instruction counts (millions), load + resample baseline subtracted on the cells side, file read on the ft8dec side
ir() {
  if [ "$(uname)" = Darwin ]; then /usr/bin/time -l "$@" 2>&1 >/dev/null | awk '/instructions retired/{print $1}'
  else valgrind --tool=callgrind --callgrind-out-file=/dev/null "$@" 2>&1 >/dev/null | awk '/Collected/{print $4}'; fi
}
m() { echo $(( ($1 - $2) / 1000000 )); }
printf "%-10s %9s %9s %9s %9s | %9s %9s %9s %9s\n" wav cells15 cells30 sic2 sic3 ft8d-1p ft8d-2p ft8d-3p ft8d-dflt
for n in qso3_busy qso1 qso2; do
  cb=$(CELLS_WAV=$n CELLS_MODE=none ir "$BIN" cells_flow_one_call_for_valgrind --ignored --test-threads=1)
  row=""; for md in c15 c30 sic2 sic3; do v=$(CELLS_WAV=$n CELLS_MODE=$md ir "$BIN" cells_flow_one_call_for_valgrind --ignored --test-threads=1); row="$row $(printf '%9s' $(m "$v" "$cb"))"; done
  fb=$(NODEC=1 ir "$W/ft8dec_drv" "$W/${n}_8k.raw")
  row="$row |"; for cfg in "30 0 1" "30 0 2" "30 0 3" "300 2 3"; do v=$(ir "$W/ft8dec_drv" "$W/${n}_8k.raw" $cfg); row="$row $(printf '%9s' $(m "$v" "$fb"))"; done
  printf "%-10s%s\n" "$n" "$row"
done
echo "(M instructions; ft8dec columns: 30 candidates, OSD off, 1/2/3 passes, and its defaults: 300 candidates, OSD on, 3 passes)"
