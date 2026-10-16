# usage: python3 -I scripts/rtltcp/replay.py RECORDING.bin PORT  (scripts/rtltcp/run.sh drives it)
# Replay a recording as an rtl_tcp server: the same bytes at the same offsets from the start.
# The start is aligned to the recording's own UTC phase modulo 15 s, so the signals' slot grid is where it was.
import socket, struct, sys, time, threading
path, port = sys.argv[1], int(sys.argv[2])
f = open(path, "rb")
t0w, n = struct.unpack("<qI", f.read(12))
chunks = []
for _ in range(n):
    t, k = struct.unpack("<dI", f.read(12)); chunks.append((t, f.read(k)))
srv = socket.socket(); srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(("127.0.0.1", port)); srv.listen(1)
print("listening", port, "chunks", n, "span", round(chunks[-1][0], 1), flush=True)
# W0: the wall time the recording's first byte maps to, on the recording's own phase modulo 15 s,
# at least 4 s from now. The caller connects just before it (the greeting is read within 10 s).
phase = t0w % 15_000_000_000
now = time.time_ns()
W0 = now + 4_000_000_000
W0 += (phase - W0) % 15_000_000_000
print("W0", W0, flush=True)
c, _ = srv.accept()
threading.Thread(target=lambda: [None for _ in iter(lambda: c.recv(4096), b"")], daemon=True).start()
# The greeting at once (the client reads it), the rest at W0 + t.
c.sendall(chunks[0][1])
t_start = time.monotonic() + (W0 - time.time_ns()) / 1e9
for t, b in chunks[1:]:
    d = t_start + t - time.monotonic()
    if d > 0: time.sleep(d)
    try: c.sendall(b)
    except OSError: break
print("done", flush=True)
c.close()
