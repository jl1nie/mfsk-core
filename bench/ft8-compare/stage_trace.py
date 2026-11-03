#!/usr/bin/env python3 -I
"""Where does jt9 decode each signal, and where does mfsk-core lose the ones it does not?

usage: stage_trace.py <dir> [file-stem ...]
  <dir>/files.txt            the WAV list
  <dir>/jt9_<stem>.out       jt9 -8 -d 3 output (plain build)
  <dir>/trace/<stem>.trace   ft8_trace.dat of the JT9_TRACE build (scripts/build_jt9_upstream.sh)
  <dir>/rtrace/<stem>.rtrace MFSK_FT8_TRACE of ft8-compare (probe patch in ft8/decode.rs)

For every message jt9 printed: the jt9 stage that decoded it, and what mfsk-core did with the
same signal. mfsk-core's outcome for a signal it did not decode is read from the candidates
near it (coarse f within FTOL Hz and dt within DTOL s of jt9's refined f1/dt):
  decoded      mfsk-core printed it
  no-cand      no coarse candidate near the signal in any round of any stage
  gate         candidates existed, every one stopped at the nsync gate
  ladder       a candidate passed the gate, BP/OSD found no valid message
  dropped      decoded inside the SIC loop and then discarded (DUPE / XSNRDROP)
"""
import re, sys, os, collections

FTOL, DTOL = 4.0, 0.45
norm = lambda m: re.sub(r"\s+", " ", m.strip()).upper()
JL = re.compile(r"^\d{6}\s+(-?\d+)\s+(-?[\d.]+)\s+(\d+)\s+~\s+(.*?)\s*$")
TAIL = re.compile(r"(\s+\?)?(\s+a\d+)?\s*$")


def jt9_out(path):
    out = []
    for l in open(path, errors="replace"):
        m = JL.match(l)
        if m:
            t = TAIL.search(m[4])
            out.append(dict(msg=norm(TAIL.sub("", m[4])), f=float(m[3]), dt=float(m[2]), snr=int(m[1]), tail=(t[0].strip() if t and t[0].strip() else "")))
    return out


def jt9_events(path):
    """One event per ft8b call: stage nzhsym, outer pass, candidate (sync,f,dt), outcome."""
    ev, nz, op, cand, last = [], 0, 0, None, None
    for l in open(path):
        t = l.split()
        if not t:
            continue
        k = t[0]
        if k == "STAGE": nz = int(t[1])
        elif k == "PASS": nz, op = int(t[1]), int(t[2])
        elif k == "CAND": cand = dict(sync=float(t[4]), f=float(t[5]), dt=float(t[6]))
        elif k == "GATE":
            last = dict(kind="gate", nz=nz, op=op, cand=cand, nsync=int(t[4]), f1=float(t[2]), dt1=float(t[3])); ev.append(last)
        elif k == "DEC":
            msg = norm(" ".join(t[11:]))
            last = dict(kind="dec", nz=nz, op=op, cand=cand, llr=int(t[3]), aptype=int(t[4]), ntype=int(t[5]), herr=int(t[6]), dmin=float(t[7]),
                        nsync=int(t[8]), f1=float(t[9]), dt1=float(t[10]), msg=msg); ev.append(last)
        elif k == "REJ" and last and last["kind"] == "dec":
            last["kind"] = "rej"
        elif k == "FAIL":
            last = dict(kind="fail", nz=nz, op=op, cand=cand, nsync=int(t[5]), f1=float(t[3]), dt1=float(t[4])); ev.append(last)
    return ev


KV = re.compile(r"(\w+)=(\S+)")


def mfsk_events(path):
    ev, cur, rnd = [], None, {}
    for l in open(path):
        head, _, rest = l.rstrip("\n").partition(" ")
        if head == "ROUND":
            st = rest.split()[0]; rnd = dict(stage=st, **{k: v for k, v in KV.findall(rest)})
        elif head == "CAND":
            st = rest.split()[0]; kv = dict(KV.findall(rest))
            cur = dict(kind="cand", stage=st, round=int(kv["round"]), f=float(kv["f"]), dt=float(kv["dt"]), score=float(kv["score"]), out=None)
            ev.append(cur)
        elif head == "GATE" and cur:
            kv = dict(KV.findall(rest)); cur.update(out="gate", nsync=int(kv["nsync"]), floor=int(kv["floor"]), f1=float(kv["f"]), dt1=float(kv["dt"]))
        elif head == "FAIL" and cur:
            kv = dict(KV.findall(rest)); cur.update(out="ladder", nsync=int(kv["nsync"]), f1=float(kv["f"]), dt1=float(kv["dt"]))
        elif head == "DEC" and cur:
            body, _, msg = rest.partition(" msg=")
            kv = dict(KV.findall(body)); cur.update(out="dec", nsync=int(kv["nsync"]), herr=int(kv["hard"]), method=kv["method"], f1=float(kv["f"]), dt1=float(kv["dt"]), msg=norm(msg))
        elif head in ("NOACC", "ACC", "REJ_UNPACK", "REJ_POLICY", "REJ_KNOWN") and cur:
            cur["acc"] = head
        elif head in ("DUPE", "XSNRDROP") and cur and cur.get("out") == "dec":
            cur["out"] = "dropped:" + head
    return ev


def near(c, f, dt):
    return abs(c["f"] - f) <= FTOL and abs(c["dt"] - dt) <= DTOL


def main():
    d = sys.argv[1]
    stems = sys.argv[2:] or [os.path.basename(l.strip())[:-4] for l in open(os.path.join(d, "files.txt"))]
    agg_found, agg_lost, lost_rows = collections.Counter(), collections.Counter(), []
    cause = collections.Counter()
    why_acc = collections.Counter()
    for n in stems:
        jo = jt9_out(f"{d}/jt9_{n}.out")
        je = jt9_events(f"{d}/trace/{n}.trace")
        me = mfsk_events(f"{d}/rtrace/{n}.rtrace")
        mdec = {}
        for e in me:
            if e["out"] == "dec" and e["msg"] not in mdec:
                mdec[e["msg"]] = e
        for s in jo:
            je_m = next((e for e in je if e["kind"] == "dec" and e["msg"] == s["msg"]), None)
            if je_m:
                lab = f"nz{je_m['nz']} pass{je_m['op']} " + (f"AP{je_m['aptype']}" if je_m["aptype"] else f"llr{je_m['llr']}") + (" BP" if je_m["ntype"] == 1 else " OSD")
                ref_f, ref_dt = je_m["f1"], je_m["dt1"]
            else:
                lab = "a7/a8 list (untraced)"
                ref_f, ref_dt = s["f"], s["dt"]
            got = mdec.get(s["msg"])
            if got and abs(got["f1"] - s["f"]) <= 10:
                agg_found[lab] += 1
                continue
            agg_lost[lab] += 1
            # what did mfsk-core do around this signal?
            nb = [c for c in me if near(c, ref_f, ref_dt)]
            passed = [c for c in nb if c["out"] in ("ladder", "dec") or (c["out"] or "").startswith("dropped")]
            if any((c["out"] or "").startswith("dropped") for c in nb): why = "dropped"
            elif passed: why = "ladder"
            elif any(c["out"] == "gate" for c in nb): why = "gate"
            else: why = "no-cand"
            cause[(lab, why)] += 1
            where = "; ".join(f"{c['stage']}{c['round']}:{c['out']}" + (f"/{c['acc']}" if c.get("acc") and c["out"] == "ladder" else "") + (f"(ns={c['nsync']})" if "nsync" in c else "") for c in nb[:6]) or "-"
            for c in nb:
                if c["out"] == "ladder" and c.get("acc"): why_acc[c["acc"]] += 1
            jc = je_m["cand"] if je_m else None
            # our refined (f, dt) at the same stage, against jt9's refined (f1, dt1)
            refine = ""
            if je_m:
                stg = "A" if je_m["nz"] == 41 else "C"
                same = [c for c in nb if c["stage"] == stg and "f1" in c]
                if same:
                    c = min(same, key=lambda c: abs(c["f1"] - ref_f))
                    refine = f"refined df={c['f1']-ref_f:+.2f}Hz ddt={c['dt1']-ref_dt:+.3f}s ns(ours/jt9)={c['nsync']}/{je_m['nsync']}"
            lost_rows.append((n, s["msg"], s["f"], s["dt"], s["snr"], lab, (f"sync={jc['sync']:.1f} herr={je_m['herr']} {refine}" if je_m else ""), why, where))
    print("jt9 decode stage            found by mfsk-core / lost")
    for lab in sorted(set(agg_found) | set(agg_lost)):
        print(f"  {lab:28s} {agg_found[lab]:4d} / {agg_lost[lab]:3d}")
    print(f"  {'total':28s} {sum(agg_found.values()):4d} / {sum(agg_lost.values()):3d}\n")
    cls = lambda lab: "AP (CQ hypothesis)" if "AP" in lab else ("OSD" if lab.endswith("OSD") else ("BP" if lab.endswith("BP") else "a7/a8 list"))
    byc = collections.defaultdict(lambda: [0, 0])
    for lab, n_ in agg_found.items(): byc[cls(lab)][0] += n_
    for lab, n_ in agg_lost.items(): byc[cls(lab)][1] += n_
    print("by the rung that decoded it in jt9:   found / lost   (lost share)")
    for k, (a, b) in sorted(byc.items()):
        print(f"  {k:20s} {a:4d} / {b:3d}   {100*b/max(a+b,1):5.1f}%")
    print()
    print("ladder outcomes of the candidates near the lost signals:", dict(why_acc))
    print("where mfsk-core lost them:", dict(collections.Counter(w for (_, w) in cause.elements())), "\n")
    print("lost signals: file | message | f dt snr(jt9) | jt9 stage | jt9 detail | mfsk-core verdict | candidates near it (stage+round:outcome)")
    for r in lost_rows:
        print(f"  {r[0]} | {r[1]} | {r[2]:.0f} {r[3]:+.1f} {r[4]:+d} | {r[5]} | {r[6]} | {r[7]} | {r[8]}")


main()
