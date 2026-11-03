#!/usr/bin/env python3 -I
"""Why does mfsk-core print a message that jt9 does not? One row per such message.

usage: extras_trace.py <dir> [stem ...]   (<dir>/rtrace4 = our trace with the fix; trace2 = jt9's)
For each message in our final output that jt9's output lacks: where we decoded it (stage, search
round, coarse candidate, refined position, nsync, rung, hard errors) and what jt9 did around the same
spot in the same stage (nz41 = our A, nz50 = our C): its candidates within 2.5 Hz and 0.6 s, by pass.
"""
import sys, os, re, glob, csv
FW, DW = 2.5, 0.6
norm = lambda m: re.sub(r"\s+", " ", m.strip()).upper()
JL = re.compile(r"^\d{6}\s+(-?\d+)\s+(-?[\d.]+)\s+(\d+)\s+~\s+(.*?)\s*$"); TAIL = re.compile(r"(\s+\?)?(\s+a\d+)?\s*$")
d = sys.argv[1]
stems = sys.argv[2:] or [os.path.basename(p)[:-4] for p in sorted(glob.glob(f"{d}/rtrace4/*.csv"))]
tot = ncand_j = ncand_o = 0
for n in stems:
    jt = {norm(TAIL.sub("", m[4])) for l in open(f"{d}/jt9_{n}.out", errors="replace") if (m := JL.match(l))}
    ours = [norm(r["message"]) for r in csv.DictReader(open(f"{d}/rtrace4/{n}.csv")) if r["decoder"] == "mfsk" and r["message"]]
    extras = [m for m in ours if m not in jt]
    if not extras: continue
    # our DEC events with the coarse candidate that produced them
    ev, cur, rnd = [], None, 0
    for l in open(f"{d}/rtrace4/{n}.rtrace"):
        h, _, r = l.rstrip("\n").partition(" ")
        if h == "CAND":
            kv = dict(re.findall(r"(\w+)=(\S+)", r)); cur = dict(stage=r.split()[0], rnd=int(kv["round"]), f=float(kv["f"]), dt=float(kv["dt"]), score=float(kv["score"]), acc=None)
        elif h == "ACC" and cur: cur["acc"] = r
        elif h == "DEC" and cur:
            body, _, msg = r.partition(" msg="); kv = dict(re.findall(r"(\w+)=(\S+)", body))
            ev.append(dict(cur, msg=norm(msg), nsync=int(kv["nsync"]), hard=int(kv["hard"]), method=kv["method"], pid=kv["pass"], rf=float(kv["f"]), rdt=float(kv["dt"])))
    # jt9 candidates by (nz, pass) with outcome
    jc, nz, op, cand = [], 0, 0, None
    for l in open(f"{d}/trace2/{n}.trace"):
        t = l.split()
        if not t: continue
        if t[0] == "PASS": nz, op = int(t[1]), int(t[2])
        elif t[0] == "CAND": cand = dict(nz=nz, op=op, sync=float(t[4]), f=float(t[5]), dt=float(t[6]))
        elif t[0] in ("GATE", "FAIL", "DEC") and cand:
            cand["out"] = t[0] + (f"(nsync={t[4]})" if t[0] == "GATE" else f"(nsync={t[5]})" if t[0] == "FAIL" else f"(herr={t[6]} llr{t[3]})")
            jc.append(cand); cand = None
    for m in extras:
        e = next((x for x in ev if x["msg"] == m), None)
        tot += 1
        if not e: print(f"{n} | {m} | (no DEC event found)"); continue
        stg = 41 if e["stage"] == "A" else 50
        near = [c for c in jc if c["nz"] == stg and abs(c["f"] - e["f"]) <= FW and abs(c["dt"] - e["dt"]) <= DW]
        kind = "free text/odd" if not re.search(r"[A-Z0-9]{3,}[ ]+[A-Z0-9/<>]{3,}", m) else "call-like"
        print(f"{n} | {m!r} [{kind}] | ours {e['stage']}{e['rnd']} coarse f={e['f']:.1f} dt={e['dt']:+.2f} score={e['score']:.2f} -> refined {e['rf']:.1f}/{e['rdt']:+.2f} nsync={e['nsync']} {e['method']} pass={e['pid']} hard={e['hard']}")
        print("      jt9 near the spot:", "; ".join(f"pass{c['op']} sync={c['sync']:.1f} f={c['f']:.1f} dt={c['dt']:+.2f} {c['out']}" for c in near) if near else "NO candidate within 2.5 Hz / 0.6 s in the same stage")
print("extras:", tot)
