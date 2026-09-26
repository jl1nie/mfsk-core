#!/usr/bin/env bash
# Generate noise-only JTTY recordings: 60 files of 30 s of Gaussian noise
# (sjtty at SNR -99 dB, i.e. the signal is ~1e-5 of the noise), for the
# "noise decodes nothing" precision test.
#
# Usage:
#   scripts/gen_jtty_noise_wavs.sh [WSJT-X-dir] [out-dir]
#
# Default out-dir embedded-poc/assets/jtty_noise/ (gitignored). Deterministic:
# sjtty seeds its own noise. 30 minutes in total; WSJT-X's rjtty decodes nothing
# in them, and neither should this crate.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WSJTX_DIR="${1:-$(cd "$REPO_ROOT/../WSJT-X" 2>/dev/null && pwd || echo "")}"
OUT_DIR="${2:-$REPO_ROOT/embedded-poc/assets/jtty_noise}"

"$SCRIPT_DIR/build_jttysim.sh" "$WSJTX_DIR" >/dev/null
TOOLS="$REPO_ROOT/target/jttysim/build"
mkdir -p "$OUT_DIR"
OUT_DIR="$(cd "$OUT_DIR" && pwd)"
printf '# sjtty-generated noise-only WAVs -- regenerate with scripts/gen_jtty_noise_wavs.sh\n*.wav\n' > "$OUT_DIR/.gitignore"

# an 80-character message makes 16 frames, so 30.2 s of audio per file
MSG="AAAAA AAAAA AAAAA AAAAA AAAAA AAAAA AAAAA AAAAA AAAAA AAAAA AAAAA AAAAA AAAAA AAAAA AA"
cd "$OUT_DIR"
# no pipe into head/tee here: sjtty would die of SIGPIPE before writing its files
"$TOOLS/sjtty" "$MSG" 1500 0.3 AW 0 384 60 -99 > /dev/null
echo "Wrote $(ls -1 ./*.wav | wc -l) noise files to $OUT_DIR"
