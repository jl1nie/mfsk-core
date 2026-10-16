# usage: python3 -I scripts/rtltcp/hole_patterns.py KIND OUT.csv [TRUTH.json]
# Loss traces for `synth_rtltcp --loss trace:OUT.csv` with the measured statistics (3.54 holes/s, 10.46 ms, resets of
# 1.43 s at 107/335/440 s) but a different arrangement (RTLTCP_EVAL_PLAN.md section 11):
#   periodic    equally spaced, all 10.46 ms
#   perlen      equally spaced, lengths drawn (random.seed(7)) from the holes of TRUTH.json (the seed-1 corpus)
#   poisconst   Poisson spacing, all 10.46 ms (draws from the random stream that perlen leaves behind, so it
#               too needs TRUTH.json; this is how the section 11 traces were made)
import json, random, sys

R = 1_024_000
kind, out = sys.argv[1:3]
random.seed(7)
rows = []
if kind == "periodic":
    rows = [f"hole,{int(i / 3.54 * R)},{round(10.46e-3 * R)}" for i in range(1, int(600 * 3.54))]
elif kind == "perlen":
    lens = [l["len"] for l in json.load(open(sys.argv[3]))["loss"] if l["kind"] == "hole"]
    rows = [f"hole,{int(i / 3.54 * R)},{random.choice(lens)}" for i in range(1, int(600 * 3.54))]
elif kind == "poisconst":
    lens = [l["len"] for l in json.load(open(sys.argv[3]))["loss"] if l["kind"] == "hole"]
    for i in range(1, int(600 * 3.54)):
        random.choice(lens)
    t = 0.0
    while True:
        t += random.expovariate(3.54)
        if t >= 600:
            break
        rows.append(f"hole,{int(t * R)},{round(10.46e-3 * R)}")
else:
    sys.exit("KIND: periodic | perlen | poisconst")
rows += [f"reset,{at * R},{round(1.43 * R)}" for at in (107, 335, 440)]
open(out, "w").write("kind,start_sample,len\n" + "\n".join(rows) + "\n")
