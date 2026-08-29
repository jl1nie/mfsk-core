#!/usr/bin/env bash
# Generate the FT4 **crowded-band** corpus with WSJT-X's own multi-signal
# simulator, `ft4sim_mult`.
#
# Usage:
#   scripts/gen_ft4_mult_wavs.sh [ft4sim_mult-path] [out-dir]
#
# Env:
#   FILES         files per occupancy (default 50, max 99 — the sim reads
#                 the file index with an `i2` format)
#   OCCUPANCIES   signals per slot (default "10 20 30")
#   SEED          RNG seed for the scene layout (default 1)
#
# Output (default `embedded-poc/assets/ft4_mult/`):
#   ft4_mult_n<occ>_<file>.wav   7.5 s, 12 kHz mono PCM-16
#   manifest.tsv                 wav \t snr_db \t dt_s \t freq_hz \t message
#
# ## Why this corpus exists
#
# `gen_ft4_sweep_wavs.sh` puts **one** signal in a slot at a fixed
# `DT = 0.0`. That is the right instrument for a sensitivity curve and
# the wrong one for every question about a crowded band, which is what
# FT4 is actually used in — it cannot see candidate counts under real
# occupancy, how deep a weak signal ranks among stronger neighbours,
# phantom decodes, or anything timing-related. Both gaps have already
# cost this project a wrong answer: `docs/notes/FT4_BENCHMARK.md` §18
# (the fixed DT made a window-narrowing test report no loss at any
# width) and §21.2 (every decode in the sweep corpus ranks 0, which is
# a property of the fixture, not a finding).
#
# `ft4sim_mult` lays N signals into one slot, each at its own SNR and
# frequency and a **random DT in ±0.5 s**, then adds Gaussian noise —
# and prints the ground truth it used, which is what `manifest.tsv`
# records. No fading: this is the crowded **AWGN** case, complementary
# to `ft4_sweep`'s CCIR channels rather than a replacement.
#
# The message pool is upstream's own `lib/ft4/messages.txt`, i.e. real
# contest traffic (and the scene that generated the WSJT-X FT4 sample
# this repo uses as its golden — its 297 Hz `N1TRK N4FKH 569 VA` row is
# in there).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

SIM="${1:-$REPO_ROOT/target/ft4sim/ft4sim_mult}"
OUT_DIR="${2:-$REPO_ROOT/embedded-poc/assets/ft4_mult}"
WSJTX_DIR="$(cd "$REPO_ROOT/../WSJT-X" 2>/dev/null && pwd || echo "")"

FILES="${FILES:-50}"
OCCUPANCIES="${OCCUPANCIES:-10 20 30}"
SEED="${SEED:-1}"

if [[ ! -x "$SIM" ]]; then
  echo "error: ft4sim_mult not found at $SIM" >&2
  echo "Run scripts/build_ft4sim.sh first." >&2
  exit 1
fi
if [[ -z "$WSJTX_DIR" || ! -f "$WSJTX_DIR/lib/ft4/messages.txt" ]]; then
  echo "error: WSJT-X tree not found (needed for the message pool)" >&2
  exit 1
fi
if (( FILES > 99 )); then
  echo "error: FILES must be <= 99 (ft4sim_mult reads the file index as i2)" >&2
  exit 1
fi

mkdir -p "$OUT_DIR"
MANIFEST="$OUT_DIR/manifest.tsv"
: > "$MANIFEST"
printf 'wav\tsnr_db\tdt_s\tfreq_hz\tmessage\n' >> "$MANIFEST"

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

echo "Generating FT4 crowded-band corpus: occupancies [$OCCUPANCIES], $FILES files each"
echo "Output: $OUT_DIR"

for OCC in $OCCUPANCIES; do
  echo "  occupancy $OCC ..."
  python3 - "$WSJTX_DIR/lib/ft4/messages.txt" "$WORK/messages.txt" "$OCC" "$FILES" "$SEED" <<'PY'
import random, sys

pool_path, out_path, occ, files, seed = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5])

# Upstream's own decode log, message column (cols 49.. in its fixed
# format). Deduplicated, order preserved.
msgs = []
for line in open(pool_path):
    line = line.rstrip("\n")
    if line.startswith("File") or len(line) < 49:
        continue
    m = line[48:85].strip()
    if m and m not in msgs:
        msgs.append(m)
if not msgs:
    sys.exit("no messages in pool")

# One RNG per (occupancy, seed) so a corpus is reproducible and the
# occupancies are not correlated scenes of each other.
rng = random.Random(seed * 1000 + occ)

# SNR ladder: dense around FT4's own threshold (-17 is the sim's floor,
# it clamps below that), thinning out towards the strong end where a
# decode is never in doubt.
LADDER = [-17, -16, -15, -14, -13, -12, -11, -10, -8, -6, -3, 0, +5, +10]

# Frequency layout: 300..2600 Hz, drawn on a jittered grid with a 15 Hz
# floor on separation. An FT4 signal occupies ~83 Hz, so neighbours
# routinely overlap here — which is the point, and what a contest band
# looks like.
FLO, FHI, FMIN_SEP = 300.0, 2600.0, 15.0

with open(out_path, "w") as f:
    for ifile in range(1, files + 1):
        f.write(f"File{ifile:2d}\n")
        freqs = []
        guard = 0
        while len(freqs) < occ and guard < 10000:
            guard += 1
            cand = rng.uniform(FLO, FHI)
            if all(abs(cand - x) >= FMIN_SEP for x in freqs):
                freqs.append(cand)
        freqs.sort()
        for isig, f0 in enumerate(freqs):
            snr = LADDER[(ifile * occ + isig) % len(LADDER)]
            msg = msgs[rng.randrange(len(msgs))]
            # `ft4sim_mult` reads f0 as `ifreq*960/576`, i.e. ifreq is
            # 0.6 x the audio frequency in Hz.
            ifreq = int(round(f0 * 576.0 / 960.0))
            # Fixed-format row: a4, 30x, i3 snr, f5.1 dt (read then
            # overwritten by the sim's own random DT), i5 ifreq, 1x, a37.
            row = "MULT" + " " * 30 + f"{snr:3d}" + f"{0.0:5.1f}" + f"{ifreq:5d}" + " " + msg
            f.write(row + "\n")
PY

  ( cd "$WORK" && "$SIM" "$OCC" "$FILES" > sim.log 2>&1 ) || {
    echo "error: ft4sim_mult failed; see $WORK/sim.log" >&2
    tail -20 "$WORK/sim.log" >&2
    exit 1
  }
  if grep -q "clipped" "$WORK/sim.log"; then
    echo "    warning: ft4sim_mult reported clipping at occupancy $OCC" >&2
  fi

  python3 - "$WORK" "$OUT_DIR" "$OCC" "$MANIFEST" <<'PY'
import os, shutil, sys

work, out_dir, occ, manifest = sys.argv[1], sys.argv[2], int(sys.argv[3]), sys.argv[4]

# `ft4sim_mult` prints one row per signal, format(a13,i4,i5,f5.1,i6,2x,a37):
#   fname(1:13), isig, isnr, xdt (the *random* one it used), nint(f0), msg
rows = []
for line in open(os.path.join(work, "sim.log")):
    line = line.rstrip("\n")
    if not line.startswith("000000_") or len(line) < 36:
        continue
    fname = line[0:13]
    rows.append((fname, int(line[17:22]), float(line[22:27]),
                 int(line[27:33]), line[35:72].strip()))

moved = set()
with open(manifest, "a") as mf:
    for fname, snr, dt, f0, msg in rows:
        idx = int(fname.split("_")[1])
        wav = f"ft4_mult_n{occ}_{idx:03d}.wav"
        src = os.path.join(work, fname + ".wav")
        if wav not in moved:
            if not os.path.exists(src):
                sys.exit(f"missing {src} — the sim did not write it")
            shutil.move(src, os.path.join(out_dir, wav))
            moved.add(wav)
        mf.write(f"{wav}\t{snr}\t{dt:.2f}\t{f0}\t{msg}\n")
print(f"    {len(moved)} files, {len(rows)} signals")
PY
done

echo ""
echo "Wrote $(ls "$OUT_DIR"/*.wav | wc -l) WAVs and $(( $(wc -l < "$MANIFEST") - 1 )) ground-truth rows"
echo "Manifest: $MANIFEST"
