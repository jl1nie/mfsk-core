#!/usr/bin/env bash
# Regenerate the JTTY oracle fixtures under embedded-poc/assets/golden/jtty/
# from WSJT-X's own sjtty / rjtty and our jtty_ladder_oracle driver
# (scripts/jttysim/), built from a clean export of the tag by
# scripts/build_jttysim.sh.
#
# Usage:
#   scripts/gen_jtty_vectors.sh [WSJT-X-dir]
#
# Writes:
#   golden/jtty/260807_134110.wav            upstream's sample recording
#   golden/jtty/260807_134110.expected.txt   rjtty's decode of it (ndebug=1)
#   golden/jtty/sim/*.wav                    sjtty recordings (AWGN, deterministic)
#   golden/jtty/sim/MANIFEST.tsv             message, profile, tones, rjtty result
#   golden/jtty/ladder_cases.txt             TBCC ladder inputs and outputs
#
# Everything is deterministic: sjtty seeds its noise (same arguments give the
# same bytes, checked), rjtty is deterministic (checked twice), and the ladder
# oracle takes an explicit seed. Rerunning this script must leave `git status`
# clean; if it does not, the oracle changed and that is the finding.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
TAG="${JTTY_TAG:-v3.2.0-rc1}"
WSJTX_DIR="${1:-$(cd "$REPO_ROOT/../WSJT-X" 2>/dev/null && pwd || echo "")}"
OUT="$REPO_ROOT/embedded-poc/assets/golden/jtty"
TOOLS="$REPO_ROOT/target/jttysim/build"

"$SCRIPT_DIR/build_jttysim.sh" "$WSJTX_DIR" >/dev/null
COMMIT="$(git -C "$WSJTX_DIR" rev-parse "$TAG^{commit}")"
mkdir -p "$OUT/sim"

# ---- 1. upstream's sample recording and what rjtty makes of it -------------
SAMPLE=260807_134110.wav
git -C "$WSJTX_DIR" show "$TAG:samples/JTTY/$SAMPLE" > "$OUT/$SAMPLE"
{
  echo "# rjtty (WSJT-X $TAG, $COMMIT) on $SAMPLE"
  echo "# sha256 $(sha256sum "$OUT/$SAMPLE" | cut -d' ' -f1)"
  echo "# command: rjtty 4.6 1 384 1500 50 $SAMPLE   (smin ndebug nsps f0 ftol)"
  echo "# columns: ichan ipass ic ndecodes iactive nactive match interferer f1 xdt tsync snr nsync nsymerrs msg"
  (cd "$OUT" && "$TOOLS/rjtty" 4.6 1 384 1500 50 "$SAMPLE" | tail -n +2)
} > "$OUT/${SAMPLE%.wav}.expected.txt"

# ---- 2. sjtty simulations --------------------------------------------------
# name | profile | message | f0 | dt | snr | keep-wav
VECTORS=(
  "cq_1frame|unknown|CQ K1ABC CQ|1500|0.3|-8|wav"
  "call_exch_2frames|unknown|WB9XYZ 599 123|1500|0.3|-8|wav"
  "text5_3frames|unknown|HELLO WORLD 73|1500|0.3|-8|wav"
  "class_section|field-day|1D EMA|1500|0.3|-8|wav"
  "rtty_serial|rtty-roundup|K1ABC 599 001|1500|0.3|-8|wav"
  "tu_call|unknown|TU K1ABC CQ|1500|0.0|-30|tones"
  "call_tu|unknown|K1ABC TU|1500|0.0|-30|tones"
  "call_agn|unknown|W9XYZ AGN?|1500|0.0|-30|tones"
  "tu_now|unknown|TU NOW W7UVW|1500|0.0|-30|tones"
  "generic_qth|unknown|599 MA|1500|0.0|-30|tones"
  "grid4_full|unknown|599 FN42|1500|0.0|-30|tones"
  "control_qsl_tu|unknown|QSL TU|1500|0.0|-30|tones"
  "numeric|unknown|123|1500|0.0|-30|tones"
  "call_with_slash|unknown|K1ABC/P|1500|0.0|-30|tones"
  "lowercase_punct|unknown|cq k1abc, hi!|1500|0.0|-30|tones"
)
: > "$OUT/sim/MANIFEST.tsv.tmp"
printf '# WSJT-X %s (%s) sjtty/rjtty\n# name\tprofile\tmessage\tf0\tdt\tsnr\tnsym\ttones\twav\trjtty_f1\trjtty_dt\trjtty_text\n' \
  "$TAG" "$COMMIT" > "$OUT/sim/MANIFEST.tsv.tmp"
WORK="$(mktemp -d)"; trap 'rm -rf "$WORK"' EXIT
for v in "${VECTORS[@]}"; do
  IFS='|' read -r name profile msg f0 dt snr keep <<<"$v"
  rm -rf "$WORK"/*; ( cd "$WORK"
    "$TOOLS/sjtty" "--exchange-profile=$profile" "$msg" "$f0" "$dt" AW 0 384 1 "$snr" > sjtty.out )
  nsym="$(sed -n 's/.*Transmission length: *[0-9.]* s, *\([0-9]*\) channel symbols.*/\1/p' "$WORK/sjtty.out")"
  # the tone lines sit between the "channel symbols:" line and the "f0:" line
  tones="$(awk '/channel symbols:/{on=1;next} /^ *f0:/{on=0} on' "$WORK/sjtty.out" | tr -s ' \n' ' ' | sed 's/^ //;s/ $//')"
  wav=-; f1=-; xdt=-; text=-
  if [[ "$keep" == wav ]]; then
    wav="$name.wav"; cp "$WORK/000000_000001.wav" "$OUT/sim/$wav"
    line="$(cd "$OUT/sim" && "$TOOLS/rjtty" 4.6 1 384 1500 50 "$wav" | awk '$1 ~ /^[0-9]+$/ && NF>=15 {l=$0} END{print l}')"
    f1="$(awk '{print $9}' <<<"$line")"; xdt="$(awk '{print $10}' <<<"$line")"
    text="$(awk '{s=$15; for(i=16;i<=NF;i++) s=s" "$i; print s}' <<<"$line")"
  fi
  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
    "$name" "$profile" "$msg" "$f0" "$dt" "$snr" "$nsym" "$tones" "$wav" "$f1" "$xdt" "$text" \
    >> "$OUT/sim/MANIFEST.tsv.tmp"
done
mv "$OUT/sim/MANIFEST.tsv.tmp" "$OUT/sim/MANIFEST.tsv"

# ---- 3. TBCC ladder oracle -------------------------------------------------
LADDER="$OUT/ladder_cases.txt"
"$TOOLS/jtty_ladder_oracle" 1 3 > "$LADDER"
# The cases are only useful if they exercise every outcome of the ladder.
awk '
  /^DEC /{ ok=($2=="T"); split($4,b,"="); half=($6=="half=T");
           if(!ok) f++; else if(half) h++; else if(b[2]==1) l1++; else if(b[2]==2) l2++; else if(b[2]==4) l4++ }
  END{ printf "ladder outcomes: L1=%d L2=%d L4=%d half=%d fail=%d\n", l1,l2,l4,h,f;
       if(!(l1&&l2&&l4&&h&&f)){ print "error: the generated cases do not exercise every ladder outcome" > "/dev/stderr"; exit 1 } }' "$LADDER"

echo "Wrote $OUT (tag $TAG, $COMMIT)"
