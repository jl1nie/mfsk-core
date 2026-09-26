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
#   golden/jtty/sim/mix_*.wav, MIXES.tsv      several signals in one recording, rjtty's messages
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
  # SNR > 90 makes sjtty write the noiseless waveform (peak-normalised): the
  # encoder/synthesiser oracle. Not a sensitivity fixture.
  "clean_cq_1frame|unknown|CQ K1ABC CQ|1500|0.3|91|wav"
  "clean_call_exch_2frames|unknown|WB9XYZ 599 123|1500|0.3|91|wav"
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

# ---- 2b. multi-signal recordings --------------------------------------------
# sjtty writes one signal per file, with its own noise. To get several signals in
# *one* noise, each component is generated noiseless (SNR 91: peak-normalised to
# 32766.9), scaled to its SNR against one noise-only recording (sjtty at SNR -99,
# noise sigma 100 units, i.e. 4167 in 2500 Hz: signal amplitude
# sqrt(2 * 4167 * 10^(snr/10))), and everything is added sample by sample
# (int16, saturating). What rjtty decodes (its ndebug-0 output, which leaves out
# frames it absorbed as repeats) is the expected result.
# name | component;component... where a component is
#   profile,message,f0,dt,fading(AW|MM|...),snr
MIXES=(
  "two_close|unknown,CQ K1ABC CQ,1500,0.3,AW,-8;unknown,WB9XYZ 599 123,1560,0.9,AW,-12"
  "strong_weak|unknown,HELLO WORLD 73,1500,0.3,AW,-3;unknown,CQ W9XYZ CQ,1530,0.6,AW,-12"
  "three_channels|rtty-roundup,K1ABC 599 001,1500,0.3,AW,-8;unknown,CQ JA6DEF CQ,1300,0.5,AW,-8;unknown,TU W7UVW CQ,1700,0.8,AW,-8"
  "fading_pair|unknown,CQ K1ABC CQ,1500,0.3,MM,-4;unknown,CQ W9XYZ CQ,1560,0.8,MM,-4"
  "back_to_back|unknown,CQ K1ABC CQ,1500,0.3,AW,-8;unknown,K1ABC TU,1503,3.3,AW,-8"
  "overlap_same_freq|unknown,HELLO WORLD 73,1500,0.3,AW,-2;unknown,CQ W9XYZ CQ,1515,1.1,AW,-12"
  "four_stations|unknown,CQ K1ABC CQ,1500,0.3,AW,-6;unknown,CQ W9XYZ CQ,1300,0.4,AW,-6;unknown,CQ JA6DEF CQ,1650,0.5,AW,-6;unknown,CQ VK3NV CQ,1750,0.6,AW,-6"
)
NOISE="$WORK/noise"; mkdir -p "$NOISE"
( cd "$NOISE" && "$TOOLS/sjtty" "AAAAA AAAAA AAAAA AAAAA AAAAA AAAAA AAAAA AAAAA AAAAA AAAAA AAAAA AAAAA AAAAA AAAAA AA" 1500 0.3 AW 0 384 1 -99 > /dev/null )
cp "$NOISE/000000_000001.wav" "$WORK/../jtty_noise_$$.wav"; NOISEWAV="$WORK/../jtty_noise_$$.wav"
printf '# WSJT-X %s (%s) sjtty mixtures in one noise, rjtty result\n# name\tcomponents\trjtty_messages (| separated, in order)\n' \
  "$TAG" "$COMMIT" > "$OUT/sim/MIXES.tsv.tmp"
for m in "${MIXES[@]}"; do
  name="${m%%|*}"; comps="${m#*|}"
  rm -rf "$WORK"/c*; n=0; args=()
  IFS=';' read -ra parts <<<"$comps"
  for c in "${parts[@]}"; do
    n=$((n+1)); IFS=',' read -r profile msg f0 dt fad snr <<<"$c"
    mkdir "$WORK/c$n"
    ( cd "$WORK/c$n" && "$TOOLS/sjtty" "--exchange-profile=$profile" "$msg" "$f0" "$dt" "$fad" 0 384 1 91 > /dev/null )
    args+=("$WORK/c$n/000000_000001.wav:$snr")
  done
  python3 - "$OUT/sim/mix_$name.wav" "$NOISEWAV" "${args[@]}" <<'PY'
import sys, wave, array, math
out, noise, *comps = sys.argv[1:]
def read(p):
    w = wave.open(p); assert w.getframerate() == 12000 and w.getsampwidth() == 2
    a = array.array('h'); a.frombytes(w.readframes(w.getnframes())); return a
sigs = []
for c in comps:
    path, snr = c.rsplit(':', 1)
    a = read(path)
    amp = math.sqrt(2 * 4167.0 * 10 ** (float(snr) / 10))   # peak amplitude for this SNR in 2500 Hz
    sigs.append((a, amp / 32766.9))
n = max(len(a) for a, _ in sigs)
nz = read(noise)
mix = array.array('h', [0]) * n
for i in range(n):
    x = nz[i] + sum(g * a[i] for a, g in sigs if i < len(a))
    mix[i] = max(-32768, min(32767, int(round(x))))
w = wave.open(out, 'wb'); w.setnchannels(1); w.setsampwidth(2); w.setframerate(12000)
w.writeframes(mix.tobytes()); w.close()
PY
  # ndebug 0: only frames that were not absorbed as repeats; keep the distinct texts
  # that are not a prefix of another (a message's earlier frames are its prefixes)
  complete="$(cd "$OUT/sim" && "$TOOLS/rjtty" 4.6 0 384 1500 50 "mix_$name.wav" \
    | awk 'NF>=2 {$1=""; sub(/^ +/,""); print}' | python3 -c '
import sys
t = []
for l in sys.stdin:
    l = l.rstrip("\n")
    if l and l not in t: t.append(l)
print("|".join(x for x in t if not any(y != x and y.startswith(x) for y in t)))')"
  printf '%s\t%s\t%s\n' "$name" "$comps" "$complete" >> "$OUT/sim/MIXES.tsv.tmp"
done
rm -f "$NOISEWAV"
mv "$OUT/sim/MIXES.tsv.tmp" "$OUT/sim/MIXES.tsv"

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
