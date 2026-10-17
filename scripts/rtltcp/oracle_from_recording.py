# usage: python3 -I scripts/rtltcp/oracle_from_recording.py RECORDING.bin rsp_clock.log OUT.bin [RATE]
# The oracle of a real rsp_tcp capture (docs/notes/RTLTCP_EVAL_PLAN.md, section 13): the recording's samples with
# silence (128, 128) put back at the holes and resets the dbg4 log names, delivered in 50 ms callbacks on an ideal
# clock. Same file format as record.py, so `recorded_iq` reads it as it reads the recording. The synth corpus's
# oracle.bin is the same thing for a known signal.
#
# Positions come from holes_from_clock_log.py (--skip 0): the source's sample 0 is the first sample of the first
# record, taken to be the recording's first sample (the log's first record carries 1 344 samples already, 1.3 ms).
# The clock is the log's own: the source position of every record (samples delivered + samples lost before it)
# against its wall time, interpolated, shifted to the recording's time base by the two start times. A single fitted
# rate would put the signals' slots off by seconds where the ADC clock wanders against the PC's.
import bisect, os, struct, subprocess, sys, tempfile

rec, log, out = sys.argv[1:4]
nominal = float(sys.argv[4]) if len(sys.argv) > 4 else 1_024_000.0
here = os.path.dirname(os.path.abspath(__file__))
with tempfile.TemporaryDirectory() as d:
    trace = os.path.join(d, "trace.csv")
    subprocess.run([sys.executable, "-I", os.path.join(here, "holes_from_clock_log.py"), log, trace, str(int(nominal))],
                   check=True, capture_output=True)
    events = [(int(a), int(b)) for k, a, b in (l.strip().split(",") for l in open(trace) if l[0] in "hr")]
events.sort()

f = open(rec, "rb")
start_wall, n = struct.unpack("<qI", f.read(12))
chunks = []
for _ in range(n):
    t, ln = struct.unpack("<dI", f.read(12))
    chunks.append((t, f.read(ln)))
greeting = chunks[0][1][:12]
assert greeting[:4] == b"RTL0", greeting
payload = bytearray(chunks[0][1][12:])
for _, b in chunks[1:]:
    payload += b
assert len(payload) % 2 == 0

oracle, pos, rp, lost = bytearray(), 0, 0, 0  # pos: source samples written, rp: recording samples read
for s, ln in events:
    take = min(max(s - pos, 0), len(payload) // 2 - rp)
    oracle += payload[2 * rp : 2 * (rp + take)]
    rp += take
    pos += take
    if pos < s:  # the recording ended before this event
        break
    oracle += b"\x80\x80" * ln
    pos += ln
    lost += ln
oracle += payload[2 * rp :]
total = len(oracle) // 2

C = [l.split() for l in open(log) if l.startswith("C")]
ft0, d0 = int(C[0][1]), int(C[0][2])
# The wall clock of the log (FILETIME, 100 ns since 1601) against the recording's start (Unix ns).
shift = (ft0 * 1e-7 - 11644473600) - start_wall * 1e-9
xs, ys, j, lost_before = [], [], 0, 0
for c in C:
    d = int(c[2]) - d0
    while j < len(events) and events[j][0] <= d + lost_before:
        lost_before += events[j][1]
        j += 1
    xs.append(d + lost_before)
    ys.append((int(c[1]) - ft0) * 1e-7 + shift)
for i in range(1, len(xs)):
    xs[i] = max(xs[i], xs[i - 1] + 1)  # strictly increasing, whatever a reset's estimate did
span = ys[-1] - ys[0]
rate = (xs[-1] - xs[0]) / span
lost_log = sum(ln for s_, ln in events)
print(f"recording {len(payload) // 2} samples, oracle {total}, lost put back {lost} of {lost_log} in the log, "
      f"{len(events)} events; mean source rate {rate:.0f} S/s over {span:.1f} s (nominal {nominal:.0f}); "
      f"log starts {shift:+.3f} s from the recording's start")


def when(p):
    """Recording seconds at which source sample `p` was made, from the log (linear beyond its ends)."""
    k = min(max(bisect.bisect_right(xs, p), 1), len(xs) - 1)
    return ys[k - 1] + (p - xs[k - 1]) * (ys[k] - ys[k - 1]) / (xs[k] - xs[k - 1])


CB = 51_200  # 50 ms at 1.024 MS/s, as the synth corpus's callback
with open(out, "wb") as o:
    n_out = 1 + (total + CB - 1) // CB
    o.write(struct.pack("<qI", start_wall, n_out))
    o.write(struct.pack("<dI", 0.0, len(greeting)) + greeting)
    for k in range(0, total, CB):
        e = min(k + CB, total)
        o.write(struct.pack("<dI", when(e) + 0.02, 2 * (e - k)) + bytes(oracle[2 * k : 2 * e]))
