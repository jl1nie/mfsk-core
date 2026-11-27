#!/usr/bin/env python3
"""qso3_busy.wav through WSJT-X's jt9, this library's host decoders, ft8dec (each option) and the cells flow (each option), against the 20 known-real signals of
`mfsk-core/tests/common/ft8_qso3.rs` (WSJT-X AP-off 8 + JTDX 18 + the JTDX AP-on extra). Prints, per configuration, hits of 20, hits of WSJT-X's 8, decodes and extras.
Usage: scripts/qso3-compare.py --jt9 PATH --ft8dec-work DIR
  --jt9          a jt9 binary (version not checked)
  --ft8dec-work  the work dir of scripts/compare-ft8dec-cells.sh (it holds ft8dec_drv and the 8 kHz raw of the vendored recordings); needs ft8dec built with -DFT8_DEBUG as
                 ft8dec_drv_dbg there (see design note section 27) for the `dec` lines
Needs cargo (the host decoders run through `cargo test --ignored qso3_compare_host`; the cells flow through hosttest/icap-app's `icap-profile`)."""
import argparse, collections, os, re, subprocess, tempfile
ap = argparse.ArgumentParser(); ap.add_argument("--jt9", required=True); ap.add_argument("--ft8dec-work", required=True); a = ap.parse_args()
R = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
W = f"{R}/embedded-poc/assets/qso3_busy.wav"
src = open(f"{R}/mfsk-core/tests/common/ft8_qso3.rs").read()
ref = [" ".join(m.split()) for m in re.findall(r'msg:\s*"([^"]+)"', src)]
REF, W8 = set(ref), set(ref[:8])
rows = collections.OrderedDict()
def add(tag, cfg, msgs):
    rows.setdefault((tag, cfg), set()).update(" ".join(m.split()) for m in msgs if m.strip())
    rows.setdefault((tag, cfg), set())
for d in (1, 2, 3):
    r = subprocess.run([a.jt9, "-8", "-d", str(d), W], capture_output=True, text=True, cwd=tempfile.gettempdir())
    add("jt9", f"jt9 -8 -d {d}", [l.split("~", 1)[1] for l in r.stdout.splitlines() if re.match(r"^\d{6}\s+-?\d+\s+[-\d.]+\s+\d+\s+~", l)])
with tempfile.NamedTemporaryFile(mode="r", suffix=".txt") as f:
    subprocess.run(["cargo", "test", "-p", "mfsk-core", "--features", "full,internal-testing", "--release", "--test", "qso3_compare_host", "--", "--ignored"],
                   cwd=R, env={**os.environ, "ZZ_OUT": f.name}, check=True, capture_output=True)
    for l in open(f.name):
        t, cfg, msg = l.rstrip("\n").split("|", 2); add("host", cfg, [msg])
raw = f"{a.ft8dec_work}/qso3_busy_8k.raw"
for mc in (15, 30, 60, 300):
    for osd in (0, 1, 2):
        for ps in (1, 2, 3):
            r = subprocess.run([f"{a.ft8dec_work}/ft8dec_drv_dbg", raw, str(mc), str(osd), str(ps)], capture_output=True, text=True)
            ms = []
            for l in r.stderr.splitlines():
                if l.startswith("  dec"):
                    p = l.split(" "); i = [k for k, x in enumerate(p) if x.startswith("df")][0]; ms.append(" ".join(p[i + 1:]))
            add("ft8dec", f"ft8dec cand {mc} osd {osd} passes {ps}", ms)
subprocess.run(["cargo", "build", "--release", "--features", "fixed-point", "--bin", "icap-profile"], cwd=f"{R}/hosttest/icap-app", check=True, capture_output=True)
for mc in (15, 30, 60):
    for nos in (True, False):
        for ps in (1, 2, 3):
            cmd = [f"{R}/target/release/icap-profile", "--s3", "--cells", "--native"] + (["--no-osd"] if nos else []) + ["--max-cand", str(mc)] + (["--passes", str(ps)] if ps > 1 else []) + ["--wav", W]
            r = subprocess.run(cmd, capture_output=True, text=True)
            add("cells", f"cells cand {mc} osd {'off' if nos else 'on'} passes {ps}", [m.group(1) for l in r.stdout.splitlines() if (m := re.match(r"^\s*\d+\.\d\s+(.+)$", l))])
print(f"reference: {len(ref)} known-real signals (WSJT-X's 8 of them)")
for (tag, cfg), ms in rows.items():
    ex = sorted(ms - REF)
    print(f"{cfg:46s} {len(ms & REF):2d}/20  (WSJT-X8 {len(ms & W8)}/8)  decodes {len(ms):2d}  extras {len(ex)}" + (f": {'; '.join(ex)}" if ex else ""))
