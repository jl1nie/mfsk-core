# usage: python3 -I scripts/rtltcp/trace_controls.py KIND TRACE.csv OUT.csv
# Controls built from a real trace (holes_from_clock_log.py output) for `synth_rtltcp --loss trace:OUT.csv`
# (RTLTCP_EVAL_PLAN.md section 12). Same number of holes, same total hole length (to rounding), same resets;
# only the arrangement changes:
#   periodic     holes equally spaced between the first and the last hole start, all of the mean length
#   reallen      equally spaced, the real lengths in their real order
#   realspace    the real start positions, all of the mean length
import sys

kind, src, out = sys.argv[1:4]
holes, resets = [], []
for line in open(src):
    if line.startswith("#") or line.startswith("kind") or not line.strip():
        continue
    k, s, n = line.strip().split(",")
    (holes if k == "hole" else resets).append((int(s), int(n)))
n = len(holes)
mean = round(sum(l for _, l in holes) / n)
first, last = holes[0][0], holes[-1][0]
equal = [first + round(i * (last - first) / (n - 1)) for i in range(n)]
if kind == "periodic":
    rows = [(s, mean) for s in equal]
elif kind == "reallen":
    rows = [(s, l) for s, (_, l) in zip(equal, holes)]
elif kind == "realspace":
    rows = [(s, mean) for s, _ in holes]
else:
    sys.exit("KIND: periodic | reallen | realspace")
with open(out, "w") as f:
    f.write("kind,start_sample,len\n")
    for s, l in rows:
        f.write(f"hole,{s},{l}\n")
    for s, l in resets:
        f.write(f"reset,{s},{l}\n")
print(f"{kind}: {len(rows)} holes, {sum(l for _, l in rows)} samples (trace: {sum(l for _, l in holes)}), {len(resets)} resets")
