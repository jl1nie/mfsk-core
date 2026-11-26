#!/usr/bin/env bash
# Instructions per stage of the CoreS3's FT8 flows on the host (valgrind --tool=callgrind), fixed-point build, OSD off, no AP pass (no fft-rustfft):
# the build the board runs. Inclusive cost of the named functions, M instructions a slot, from `hosttest/icap-app`'s `icap-profile`.
#
#   old     the shipped S3: 12 kHz, sync8 search on the spectrogram, fine sync, refine, Goertzel stage 3    (`--s3`)
#   cells   the cells flow: 8 kHz, ft8_lib search on half-dB cells, stage 3 from the cells                  (`--s3 --native --cells`)
#   sic2/3  cells + ft8dec's subtraction passes, 2 / 3 passes in all                                        (`--passes N`)
#
# "producer" = work the board does while the slot is captured (the spectrogram / the FFT of the cells); the rest is after the emit point.
# Names are matched as substrings of callgrind's function names; a function that was not found prints "-".
# Usage: scripts/s3-stage-instr.sh [wav ...]   (default: qso3_busy qso1 qso2 from embedded-poc/assets)
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
A="$ROOT/embedded-poc/assets"
W="$(mktemp -d)"; [ -n "${KEEP:-}" ] || trap 'rm -rf "$W"' EXIT; [ -z "${KEEP:-}" ] || echo "kept: $W"
FILES=("$@"); [ ${#FILES[@]} -gt 0 ] || FILES=(qso3_busy qso1 qso2)
command -v valgrind >/dev/null && command -v callgrind_annotate >/dev/null || { echo "needs valgrind and callgrind_annotate"; exit 1; }
(cd "$ROOT/hosttest/icap-app" && cargo build --release --no-default-features --features fixed-point --bin icap-profile 2>&1 | tail -1)
BIN="$ROOT/target/release/icap-profile"; cp "$BIN" "$W/bin"

# tag|flags
CFG=("old15|--s3 --max-cand 15" "old30|--s3 --max-cand 30"
     "cells15|--s3 --native --cells --max-cand 15" "cells30|--s3 --native --cells --max-cand 30"
     "sic2|--s3 --native --cells --passes 2 --max-cand 30" "sic3|--s3 --native --cells --passes 3 --max-cand 30")
for f in "${FILES[@]}"; do for c in "${CFG[@]}"; do echo "${c%%|*} $f ${c#*|}"; done; done |
  xargs -P4 -L1 bash -c 'tag=$0; f=$1; shift 1; valgrind --tool=callgrind --callgrind-out-file='"$W"'/$tag.$f.out '"$W"'/bin "$@" --no-osd --wav '"$A"'/$f.wav >'"$W"'/$tag.$f.txt 2>/dev/null'

python3 - "$W" "${FILES[@]}" <<'E'
import subprocess, sys, re, os
W, files = sys.argv[1], sys.argv[2:]
STAGES = {
 "old":   [("total","profile::run_slot_s3 "),("search","coarse_sync::coarse_sync_gated::<"),("producer: spectrogram","spectrogram::spectrogram_cells::<"),
           ("fine sync","fine_sync_at"),("refine","refine_candidates_with"),("stage 3","process_candidates_into_with_cs_scratch_tuned_with_fill"),
           ("  goertzel","fill_symbol_spectra_goertzel_at"),("  BP","bp_decode_generic_nms_with_scratch")],
 "cells": [("total","profile::run_slot_s3_cells"),("search","coarse_lib::search_impl"),("producer: FFT","Plan2560Sc16>::process"),
           ("band_best","coarse_lib::band_best"),("stage 3","CellsWorker>::decode_round"),("  BP","bp_decode_generic_nms_with_scratch")],
 "sic":   [("total","profile::run_slot_s3_sic"),("search","coarse_lib::search_impl"),("producer: FFT (pass 0)","Plan2560Sc16>::process"),
           ("passes' search","search_near_into"),("stage 3 (all rounds)","CellsWorker>::decode_round"),("  BP","bp_decode_generic_nms_with_scratch"),
           ("subtract: sig_sub","Tabs>::sig_sub"),("subtract: sig_corr","Tabs>::sig_corr")],
}
def incl(path):
    out = subprocess.run(["callgrind_annotate","--inclusive=yes","--threshold=100",path],capture_output=True,text=True).stdout
    rows = [(int(m.group(1).replace(",","")), m.group(2)) for l in out.splitlines() if (m := re.match(r"\s*([\d,]+) \(\s*[\d.]+%\)\s+(.*)", l))]
    return rows
for tag in ["old15","old30","cells15","cells30","sic2","sic3"]:
    kind = tag.rstrip("0123456789")
    print(f"\n### {tag}  (M instructions a slot, inclusive)")
    print("| stage | " + " | ".join(files) + " |"); print("|---|" + "---|"*len(files))
    cols = []
    for f in files:
        rows = incl(f"{W}/{tag}.{f}.out")
        col = []
        for _, pat in STAGES[kind]:
            v = [n for n, name in rows if pat in name]
            col.append(f"{max(v)/1e6:.0f}" if v else "-")
        dec = sum(1 for l in open(f"{W}/{tag}.{f}.txt") if re.match(r"\s*\d", l))
        col.append(str(dec)); cols.append(col)
    for i, (label, _) in enumerate(STAGES[kind] + [("decodes", "")]):
        print(f"| {label} | " + " | ".join(c[i] for c in cols) + " |")
E
