# usage: python3 -I scripts/rtltcp/eval_synth.py CORPUS_DIR OUT_DIR SEEDS JOB [JOB ...] [--jobs N]
# Runs recorded_iq::decode_recorded_iq on the synth_rtltcp corpus and prints the section 4 scores
# (docs/notes/RTLTCP_EVAL_PLAN.md). SEEDS: "1,2,3". JOB: NAME=FILE[,ENV=VALUE ...], FILE one of clean, oracle,
# delivered; ENV is passed to the test (MFSK_RTL_EFFECTIVE_RATE=payload takes the truth's mean payload rate).
# Rows go to OUT_DIR/NAME.seedN.json; a row file that exists is reused, so delete it to re-run a job.
import json, os, subprocess, sys, concurrent.futures as cf

args = sys.argv[1:]
njobs = 3
if "--jobs" in args:
    i = args.index("--jobs")
    njobs = int(args[i + 1])
    del args[i : i + 2]
corpus, out, seeds = args[0], args[1], [int(s) for s in args[2].split(",")]
jobs = []
for spec in args[3:]:
    name, rest = spec.split("=", 1)
    parts = rest.split(",")
    jobs.append((name, parts[0], dict(p.split("=", 1) for p in parts[1:])))
os.makedirs(out, exist_ok=True)
build = subprocess.run(
    ["cargo", "test", "-p", "skimmer-core", "--release", "--lib", "--no-run", "--message-format=json"],
    capture_output=True, text=True, check=True)
exe = [json.loads(l)["executable"] for l in build.stdout.splitlines()
       if l.startswith("{") and json.loads(l).get("executable") and json.loads(l)["target"]["name"] == "skimmer_core"][-1]
CH = [("FT8", 7074000), ("FT8", 7041000), ("FT4", 7047500)]


def run(name, f, env, seed):
    d = f"{corpus}/seed{seed}"
    rows = f"{out}/{name}.seed{seed}.json"
    if not os.path.exists(rows):
        e = dict(os.environ, MFSK_RTL_RECORDING=f"{d}/{f}.bin", MFSK_RTL_OUTPUT=rows, MFSK_RTL_SKIP="0.001",
                 MFSK_RTL_END="1000", MFSK_RTL_CENTER="7016000",
                 MFSK_RTL_CHANNELS=",".join(f"{m.lower()}:{h}" for m, h in CH))
        for k, v in env.items():
            if v == "payload":
                t = json.load(open(f"{d}/truth.json"))
                v = str(t["files"]["delivered"]["payload_bytes"] / 2 / t["params"]["seconds"])
            e[k] = v
        subprocess.run([exe, "decode_recorded_iq", "--ignored", "--nocapture"], env=e, check=True,
                       capture_output=True)
    return name, seed, rows


with cf.ThreadPoolExecutor(njobs) as ex:
    done = list(ex.map(lambda a: run(*a), [(n, f, e, s) for n, f, e in jobs for s in seeds]))
for name, seed, rows in done:
    for m, h in CH:
        r = subprocess.run([sys.executable, "-I", os.path.join(os.path.dirname(__file__), "score_synth.py"),
                            f"{corpus}/seed{seed}/truth.json", rows, "--dial", str(h), "--mode", m],
                           capture_output=True, text=True, check=True).stdout.splitlines()
        print(f"[{name} seed{seed} {m} {h}]", r[1].strip())
        print("   ", r[2].strip(), "|", r[3].strip())
