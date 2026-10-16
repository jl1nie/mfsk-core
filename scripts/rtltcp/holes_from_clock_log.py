# usage: python3 -I scripts/rtltcp/holes_from_clock_log.py rsp_clock.log OUT.csv [RATE] [--skip SECONDS]
# The real loss sequence of an rsp_tcp dbg4 run, for `synth_rtltcp --loss trace:OUT.csv` (docs/notes/RTLTCP_EVAL_PLAN.md).
#
# dbg4 (scripts/rtltcp/rsp_tcp_dbg3_to_dbg4.patch) writes a pair per record: C <filetime> <samples delivered so far>
# <callbacks> <reset> <fsChanged>, then F <filetime> <firstSampleNum> <gap> <filled so far>. A record is written at
# least every 50 ms, at every reset and at every hole, so every hole is in the log: `gap` samples are missing just
# before sample `firstSampleNum`. Within one numbering (between resets) hole positions are exact.
#
# A reset invalidates the numbering. Its length is estimated from the wall clock: the samples the ADC made between the
# record before and the reset record (filetime difference x RATE) less those delivered meanwhile. Callback sizes are not
# logged; the reset's position is exact when the callbacks are of equal size (the API's usual), else off by their
# difference (well under 0.1 % of a 1.4 s reset).
# The source's sample 0 is the first sample of the first record after --skip.
import sys

args = sys.argv[1:]
skip = 0.0
if "--skip" in args:
    i = args.index("--skip")
    skip = float(args[i + 1])
    del args[i : i + 2]
log, out = args[0], args[1]
rate = float(args[2]) if len(args) > 2 else 1_024_000.0

pairs, c = [], None
for line in open(log):
    p = line.split()
    if len(p) == 6 and p[0] == "C":
        c = (int(p[1]), int(p[2]), int(p[4]))
    elif len(p) == 5 and p[0] == "F" and c is not None:
        pairs.append((c[0], c[1], c[2], int(p[2]), int(p[3])))
        c = None
if not pairs:
    sys.exit("no C/F record pairs: is this a dbg4 log?")

ft_start = pairs[0][0] + int(skip * 1e7)
pairs = [p for p in pairs if p[0] >= ft_start]
events, off, last_raw, wraps, prev = [], None, None, 0, None
for ft, delivered, reset, fsn_raw, gap in pairs:
    if prev is None:
        off, last_raw = -fsn_raw, fsn_raw
        prev = (ft, delivered, fsn_raw)
        continue
    if reset:
        p_ft, p_delivered, p_fsn = prev
        # Where the stream had got to before the reset: the previous record's sample, plus what was delivered since.
        end_before = p_fsn + wraps * 2**32 + off + (delivered - p_delivered)
        made = (ft - p_ft) * 1e-7 * rate
        lost = max(0, round(made - (delivered - p_delivered)))
        events.append(("reset", end_before, lost))
        wraps, last_raw = 0, fsn_raw
        off = end_before + lost - fsn_raw
    else:
        if fsn_raw < last_raw and last_raw - fsn_raw > 2**31:
            wraps += 1
        last_raw = fsn_raw
        if gap > 0:
            events.append(("hole", fsn_raw + wraps * 2**32 + off - gap, gap))
    prev = (ft, delivered, fsn_raw + 0)

span = (pairs[-1][0] - pairs[0][0]) * 1e-7
with open(out, "w") as f:
    f.write(f"# from {log}; rate {rate:.0f}; skip {skip} s; span {span:.1f} s; reset lengths estimated from the wall clock\n")
    f.write("kind,start_sample,length_samples\n")
    for k, s, n in events:
        if s >= 0 and n > 0:
            f.write(f"{k},{s},{n}\n")
holes = [e for e in events if e[0] == "hole"]
resets = [e for e in events if e[0] == "reset"]
print(f"{len(holes)} holes, {sum(e[2] for e in holes)} samples; {len(resets)} resets, "
      f"{sum(e[2] for e in resets)} samples (estimated); span {span:.1f} s -> {out}")
