# Compare skimmer CLI logs of replays of one recording (#686): decodes, the spread of the same station's
# frequency from slot to slot (the median, over stations decoded at least three times, of the standard
# deviation of their frequency: what a wandering rate offset shows as), and the median DT per decoded slot.
# usage: python3 -I scripts/rtltcp/compare.py OUT_DIR/out-a.log OUT_DIR/out-b.log ...
import re, statistics as st, sys

for path in sys.argv[1:]:
    slots, freq, n = {}, {}, 0
    for line in open(path):
        m = re.match(r"^(\d{6}) FT8\s+(\d+)\s+(-?\d+)\s+(-?[\d.]+)\s+(\S+) (\S+)", line)
        if m:
            n += 1
            slots.setdefault(m.group(1), []).append(float(m.group(4)))
            # The second word of a standard message is its sender.
            freq.setdefault(m.group(6), []).append(int(m.group(2)))
    sds = [st.pstdev(f) for f in freq.values() if len(f) >= 3]
    print(path)
    print("  decodes %d in %d slots; frequency spread %s" % (
        n, len(slots), "%.1f Hz" % st.median(sds) if sds else "n/a"))
    print("  DT per slot:", [round(st.median(d), 2) for _, d in sorted(slots.items())])
