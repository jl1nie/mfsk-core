#!/usr/bin/env python3
"""Tabulate JTTY pattern runs (#499): `CASE <pattern> <trial> <msg|msg…>` lines, from the CoreS3's
`jtty-bench` part 8 log and/or the host's `jtty_board_patterns` test, per pattern: messages
expected and found, unexpected ones, and — given both — trials where board and host differ.

    scripts/jtty_board_stats.py host.txt [board.log]
"""
import re, sys, collections

EXPECT = {
    "long message": ["RAN ALL NIGHT ON BAND NOISE"],
    "two stations": ["CQ K1ABC CQ", "CQ W9XYZ CQ"],
    "noise only": [],
}

def expected(pattern):
    for k, v in EXPECT.items():
        if pattern.startswith(k):
            return v
    return ["CQ K1ABC CQ"]

def load(path):
    cases, timing = {}, {}
    for line in open(path, errors="replace"):
        line = re.sub(r"\x1b\[[0-9;]*m", "", line)
        i = line.find("CASE\t")
        if i < 0:
            continue
        f = line[i:].rstrip("\n").split("\t")
        pattern, trial, msgs = f[1], int(f[2]), [m for m in f[3].split("|") if m]
        cases[(pattern, trial)] = msgs
        if len(f) > 4:
            timing[(pattern, trial)] = [float(x) for x in f[4:]]
    return cases, timing

def table(name, cases, timing):
    by = collections.OrderedDict()
    for (p, t), msgs in cases.items():
        e = expected(p)
        s = by.setdefault(p, [0, 0, 0, 0, []])
        s[0] += len(e); s[1] += sum(m in msgs for m in e); s[2] += sum(m not in e for m in msgs); s[3] += 1
        if (p, t) in timing:
            s[4].append(timing[(p, t)])
    print(f"\n== {name}")
    hdr = f"{'pattern':<36}{'found':>10}{'unexpected':>11}"
    if any(s[4] for s in by.values()):
        hdr += f"{'front ms':>10}{'back ms':>9}{'back max':>9}{'time/audio':>10}"
    print(hdr)
    tf = te = tu = 0
    for p, (e, f, u, n, tm) in by.items():
        tf += f; te += e; tu += u
        row = f"{p:<36}{f'{f}/{e}':>10}{u:>11}"
        if tm:
            front = sum(x[0] for x in tm) / len(tm)
            back = sum(x[1] for x in tm) / len(tm)
            bmax = max(x[2] for x in tm)
            rtf = max(x[3] for x in tm)
            row += f"{front:>10.0f}{back:>9.0f}{bmax:>9.0f}{rtf:>10.2f}"
        print(row)
    print(f"{'total':<36}{f'{tf}/{te}':>10}{tu:>11}")

host, _ = load(sys.argv[1])
table("host", host, {})
if len(sys.argv) > 2:
    board, timing = load(sys.argv[2])
    table("board", board, timing)
    common = set(host) & set(board)
    diff = [k for k in sorted(common) if sorted(host[k]) != sorted(board[k])]
    print(f"\nboard vs host: {len(common)} trials compared, {len(diff)} differ")
    for k in diff[:20]:
        print(f"  {k[0]} #{k[1]}: host {host[k]}  board {board[k]}")
