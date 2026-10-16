# usage: python3 -I scripts/rtltcp/score_synth.py TRUTH.json ROWS.json [ROWS.json ...] [--dial HZ] [--mode FT8]
# Score decodes of a synth_rtltcp capture against its truth (docs/notes/RTLTCP_EVAL_PLAN.md, section 4).
# ROWS.json: the MFSK_RTL_OUTPUT of recorded_iq::decode_recorded_iq. Prints, per file: decoded per SNR, SNR50
# (interpolated 50 % point), slot-UTC correctness, false decodes, and the median errors of SNR, DT and frequency.
import json, statistics as st, sys

args = sys.argv[1:]
opts = {"--dial": None, "--mode": None}
files = []
while args:
    a = args.pop(0)
    if a in opts:
        opts[a] = args.pop(0)
    else:
        files.append(a)
truth_path, rows_paths = files[0], files[1:]
truth = json.load(open(truth_path))
dial = int(opts["--dial"]) if opts["--dial"] else None


def snr50(curve):
    """Linear interpolation of the 50 % point on a curve [(snr, fraction)] sorted by SNR."""
    for (s0, f0), (s1, f1) in zip(curve, curve[1:]):
        if f0 < 0.5 <= f1:
            return s0 + (0.5 - f0) * (s1 - s0) / (f1 - f0)
    return None if not curve or curve[-1][1] < 0.5 else curve[0][0]


for path in rows_paths:
    out = json.load(open(path))
    d = dial or int(out.get("receive_dial", 0))
    txs = [t for t in truth["transmissions"] if t["dial_hz"] == d and (not opts["--mode"] or t["mode"] == opts["--mode"])]
    lo, hi = out.get("skip_s", 0), out.get("end_s", 1e9)
    t0 = truth["stream"]["start_utc_ns"] / 1e9
    # Only transmissions that lie wholly inside the decoded span count as opportunities.
    frame = {"FT8": 13.2, "FT4": 5.6}
    txs = [t for t in txs if t["slot_utc_s"] - t0 >= lo and t["slot_utc_s"] - t0 + frame[t["mode"]] <= hi]
    by_key = {(t["text"], round(t["slot_utc_s"])): t for t in txs}
    hit, wrong_slot, false = {}, 0, []
    for r in out["rows"]:
        utc = r["utc"] / 1e9 if r.get("utc") is not None else None
        t = by_key.get((r["text"], round(utc))) if utc is not None else None
        if t:
            hit[(t["text"], round(t["slot_utc_s"]))] = r
        elif any(x["text"] == r["text"] for x in txs):
            wrong_slot += 1
        else:
            false.append(r["text"])
    curve = []
    for s in sorted({t["snr_db"] for t in txs}):
        n = [t for t in txs if t["snr_db"] == s]
        k = sum((t["text"], round(t["slot_utc_s"])) in hit for t in n)
        curve.append((s, k / len(n), k, len(n)))
    errs = {"snr": [], "dt": [], "hz": []}
    for t in txs:
        r = hit.get((t["text"], round(t["slot_utc_s"])))
        if r:
            errs["snr"].append(r["snr"] - t["snr_db"])
            errs["dt"].append(r["dt"] - t["dt_s"])
            errs["hz"].append(r["frequency"] - t["rf_hz"])
    med = lambda v: "%+.2f" % st.median(v) if v else "-"
    s50 = snr50([(s, f) for s, f, _, _ in curve])
    print(f"{path}")
    print(f"  dial {d}: {len(hit)}/{len(txs)} decoded, SNR50 {'%.1f dB' % s50 if s50 is not None else 'not reached'},"
          f" wrong slot {wrong_slot}, false {len(false)}, reanchors {len(out.get('reanchors', []))}, gaps {out.get('gaps', 0)}")
    print("  " + " ".join(f"{s:+.0f}:{k}/{n}" for s, _, k, n in curve))
    print(f"  median error: SNR {med(errs['snr'])} dB, DT {med(errs['dt'])} s, freq {med(errs['hz'])} Hz")
    if false:
        print("  false:", ", ".join(false[:10]))
