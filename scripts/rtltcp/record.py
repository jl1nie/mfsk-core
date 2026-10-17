# usage: python3 -I scripts/rtltcp/record.py HOST PORT OUT.bin SECONDS RATE_HZ CENTRE_HZ
# Sends the sample rate, the frequency and (after 0.6 s) automatic gain, as the skimmer does. File: <q start
# wall ns><I chunks> then per read <d seconds since start><I length><bytes>, the greeting first.
# Record an rtl_tcp stream (greeting included) with the arrival time of every recv, as the client sees it.
import socket, struct, sys, time
host, port, out, seconds = sys.argv[1], int(sys.argv[2]), sys.argv[3], float(sys.argv[4])
rate, freq = int(sys.argv[5]), int(sys.argv[6])
s = socket.create_connection((host, port), timeout=10)
s.settimeout(None)
def cmd(c, p): s.sendall(bytes([c]) + struct.pack(">I", p))
cmd(2, rate); cmd(1, freq)
# Wall clock, not time.monotonic(): the WSL2 monotonic clock ran 6.4 % fast against NTP on 2026-10-17 (60.06 s against
# 56.46 s), and a recording timed with it disagreed with the server's own log by the same factor.
t0w = time.time_ns(); now = lambda: (time.time_ns() - t0w) * 1e-9
sent_gain = False
chunks = []
buf = bytearray(1 << 20)
while now() < seconds:
    k = s.recv_into(buf)
    if k == 0: break
    chunks.append((now(), bytes(buf[:k])))
    if not sent_gain and now() > 0.6:
        cmd(3, 0); sent_gain = True
with open(out, "wb") as f:
    f.write(struct.pack("<qI", t0w, len(chunks)))
    for t, b in chunks:
        f.write(struct.pack("<dI", t, len(b))); f.write(b)
print("chunks", len(chunks), "bytes", sum(len(b) for _, b in chunks), "start_wall_ns", t0w)
