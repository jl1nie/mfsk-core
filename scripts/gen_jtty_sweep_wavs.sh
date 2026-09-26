#!/usr/bin/env bash
# Generate sjtty WAV files for the JTTY SNR sweep (AWGN and one ITU fading
# channel), and score WSJT-X's own rjtty on them.
#
# Usage:
#   scripts/gen_jtty_sweep_wavs.sh [WSJT-X-dir] [out-dir]
#
# File naming:  jtty_<channel>_<snr>_<trial>.wav
#   channel:  awgn | mid_moderate   (ITU mid-latitude moderate: 0.5 Hz / 1 ms)
#   snr:      m16 = -16 dB, etc.  (sjtty's SNR convention; see BW in its banner)
#   trial:    01..TRIALS
#
# Outputs in <out-dir> (default embedded-poc/assets/jtty_sweep/):
#   *.wav               gitignored, regenerate with this script
#   UPSTREAM_RECALL.tsv committed: what rjtty makes of the corpus -- correct
#                       decodes and unexpected decodes per cell. It is the
#                       baseline the Rust port is compared against.
#
# Build the tools first (or let this script do it): scripts/build_jttysim.sh.
# Deterministic: sjtty seeds its own noise, so the same arguments give the
# same bytes; a cell is skipped only when every trial is already there.
# TRIALS env var (default 20) and JOBS (default nproc) as for the other
# generators. Generate into a separate out-dir rather than in place if you
# change TRIALS: a cell is drawn in one sequence, so changing TRIALS changes
# every trial in it.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WSJTX_DIR="${1:-$(cd "$REPO_ROOT/../WSJT-X" 2>/dev/null && pwd || echo "")}"
OUT_DIR="${2:-$REPO_ROOT/embedded-poc/assets/jtty_sweep}"
JOBS="${JOBS:-$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 1)}"
TRIALS="${TRIALS:-20}"

"$SCRIPT_DIR/build_jttysim.sh" "$WSJTX_DIR" >/dev/null
TOOLS="$REPO_ROOT/target/jttysim/build"
mkdir -p "$OUT_DIR"
OUT_DIR="$(cd "$OUT_DIR" && pwd)"

MSG="CQ K1ABC CQ"
F0=1500
DT=0.3

# rjtty on sjtty AWGN (measured 2026-09-26, 20 trials, `CQ K1ABC CQ`):
# 0/20 at -20, 1/20 at -18, 11/20 at -16, 20/20 at -14. The grid brackets the
# 50 % crossing on both sides so it is observed rather than censored.
SNRS="-20 -19 -18 -17 -16 -15 -14 -13 -12"
CHANNELS=( "awgn AW 0" "mid_moderate MM 0" )   # name, sjtty fdop/model, delay (unused for ITU)

snr_tag() { local s=$1; if (( s < 0 )); then printf "m%02d" "$(( -s ))"; else printf "p%02d" "$s"; fi; }

run_cell() {
  local chan=$1 model=$2 del=$3 snr=$4 tag; tag="$(snr_tag "$snr")"
  local missing=0 T
  for T in $(seq 1 "$TRIALS"); do
    [[ -f "$OUT_DIR/jtty_${chan}_${tag}_$(printf '%02d' "$T").wav" ]] || missing=1
  done
  (( missing == 0 )) && return 0
  printf "  JTTY %-13s SNR=%4d dB  generating %d files ...\n" "$chan" "$snr" "$TRIALS"
  local tmpd; tmpd="$(mktemp -d)"
  ( cd "$tmpd"
    "$TOOLS/sjtty" "$MSG" "$F0" "$DT" "$model" "$del" 384 "$TRIALS" "$snr" >/dev/null
    for T in $(seq 1 "$TRIALS"); do
      mv "$(printf '000000_%06d.wav' "$T")" "$OUT_DIR/jtty_${chan}_${tag}_$(printf '%02d' "$T").wav"
    done )
  rm -rf "$tmpd"
}
export -f run_cell snr_tag
export TOOLS OUT_DIR TRIALS MSG F0 DT

CELLS=()
for spec in "${CHANNELS[@]}"; do
  read -r chan model del <<<"$spec"
  for snr in $SNRS; do CELLS+=("$chan $model $del $snr"); done
done
echo "Generating JTTY sweep corpus: ${#CELLS[@]} cells, TRIALS=$TRIALS, JOBS=$JOBS"
printf '%s\n' "${CELLS[@]}" | xargs -P "$JOBS" -L1 bash -c 'run_cell $0 $1 $2 $3'

# ---- score rjtty ------------------------------------------------------------
# rjtty is run in the WAV's directory (it truncates long path names). A file
# counts as correct when one of its decoded lines is exactly $MSG; every other
# non-empty decoded line is an "unexpected" decode.
score_one() {
  local f=$1 lines
  lines="$(cd "$OUT_DIR" && "$TOOLS/rjtty" 4.6 0 384 1500 50 "$f" | sed 's/^ *[0-9]* *//' | sed '/^$/d')"
  local ok=0 extra=0 l
  while IFS= read -r l; do
    [[ -z "$l" ]] && continue
    if [[ "$l" == "$MSG" ]]; then ok=1; else extra=$((extra+1)); fi
  done <<<"$lines"
  printf '%s\t%s\t%s\n' "$f" "$ok" "$extra"
}
export -f score_one
TSV="$OUT_DIR/UPSTREAM_RECALL.tsv"
{
  printf '# rjtty 4.6 0 384 1500 50 over the sjtty corpus (message "%s", f0=%s, dt=%s, TRIALS=%s)\n' "$MSG" "$F0" "$DT" "$TRIALS"
  printf '# channel\tsnr_db\ttrials\tcorrect\tunexpected_decodes\n'
  ( cd "$OUT_DIR" && ls jtty_*.wav ) | xargs -P "$JOBS" -I{} bash -c 'score_one {}' | sort \
    | awk -F'\t' '{ split($1,p,"_"); n=split($1,q,"_");
                    chan=$1; sub(/^jtty_/,"",chan); sub(/_[mp][0-9][0-9]_[0-9][0-9]\.wav$/,"",chan);
                    tag=q[n-1]; snr=substr(tag,2)+0; if (substr(tag,1,1)=="m") snr=-snr;
                    key=chan"\t"snr; tr[key]++; ok[key]+=$2; ex[key]+=$3 }
               END { for (k in tr) print k "\t" tr[k] "\t" ok[k] "\t" ex[k] }' | sort -t$'\t' -k1,1 -k2,2n
} > "$TSV"
echo "Wrote $TSV"
cat "$TSV"
