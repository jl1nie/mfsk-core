#!/usr/bin/env bash
# Append one row to docs/notes/AGENT_LOG.md: how a task went, with the token numbers measured
# from the session transcript (docs/notes/AGENT_WORKFLOW.md §7). Run it when a task is done,
# after scripts/ci-wait.sh said CI PASS.
#
#   scripts/agent-log.sh <label> <port|design|port+design> [--base REV] [--session FILE]
#                        [--gate-first yes|no] [--review "found X" | none | n/a]
#                        [--fixes-after-push N] [--corrections N] [--note TEXT]
#   scripts/agent-log.sh --summary [SINCE_REV]    per-mode summary of the rows since SINCE_REV
#
# Measured (not asked): date, commits since --base (pass the commit before the task's first one;
# default HEAD~1), tokens per model from the transcript and its subagents. Not measured, so given as flags and
# left "?" when omitted: gate first time, review result, fixes after push, user corrections.
# Tokens: input + cache-creation + output per model, cache reads in their own column. A message
# appears several times in a transcript (one line per content block); it is counted once.
# --session defaults to the newest transcript of this project; subagents/ beside it is included.
set -uo pipefail
cd "$(git rev-parse --show-toplevel)"
LOG=docs/notes/AGENT_LOG.md
PROJ="$HOME/.claude/projects/$(pwd | sed 's#/#-#g')"

if [ "${1:-}" = "--summary" ]; then
  python3 -I - "$LOG" "${2:-}" <<'PY'
import sys, re, subprocess, collections
log, since = sys.argv[1], sys.argv[2]
cut = ""
if since:
    cut = subprocess.run(["git","log","-1","--format=%cs",since],capture_output=True,text=True).stdout.strip()
rows = []
for l in open(log):
    c = [x.strip() for x in l.strip().strip("|").split("|")]
    if len(c) >= 11 and re.match(r"\d{4}-\d\d-\d\d", c[0]) and c[0] >= cut:
        rows.append(c)
by = collections.defaultdict(list)
for r in rows: by[r[2]].append(r)
print(f"agent log rows since {cut or 'the start'}: {len(rows)}")
for m, rs in sorted(by.items()):
    g = [r[6] for r in rs]
    print(f"  {m}: {len(rs)} tasks; gate first time yes={g.count('yes')} no={g.count('no')} ?={g.count('?')}; "
          f"review found a defect in {sum(1 for r in rs if r[7] not in ('none','n/a','?'))}")
PY
  exit 0
fi

LABEL="${1:-}"; MODE="${2:-}"
[ -n "$LABEL" ] && [ -n "$MODE" ] || { sed -n '2,/^set -uo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'; exit 2; }
shift 2
case "$MODE" in port|design|port+design) ;; *) echo "agent-log.sh: mode must be port, design or port+design" >&2; exit 2 ;; esac
BASE=HEAD~1; SESSION=""; GATE='?'; REVIEW='?'; FIXES='?'; CORR='?'; NOTE=""
while [ $# -gt 0 ]; do
  case "$1" in
    --base) BASE="$2"; shift 2 ;;
    --session) SESSION="$2"; shift 2 ;;
    --gate-first) GATE="$2"; shift 2 ;;
    --review) REVIEW="$2"; shift 2 ;;
    --fixes-after-push) FIXES="$2"; shift 2 ;;
    --corrections) CORR="$2"; shift 2 ;;
    --note) NOTE="$2"; shift 2 ;;
    *) echo "agent-log.sh: unknown option '$1'" >&2; exit 2 ;;
  esac
done
[ -n "$SESSION" ] || SESSION=$(ls -t "$PROJ"/*.jsonl 2>/dev/null | head -n1)
[ -f "$SESSION" ] || { echo "agent-log.sh: no transcript (pass --session FILE)" >&2; exit 1; }
COMMITS=$(git rev-list --count "$BASE"..HEAD 2>/dev/null || echo '?')

TOK=$(python3 -I - "$SESSION" <<'PY'
import sys, json, glob, os, collections
f = sys.argv[1]
files = [f] + glob.glob(os.path.splitext(f)[0] + "/subagents/*.jsonl")
best = {}
for p in files:
    for l in open(p, errors="replace"):
        try: d = json.loads(l)
        except ValueError: continue
        m = d.get("message")
        if d.get("type") != "assistant" or not isinstance(m, dict) or not m.get("usage"): continue
        u = m["usage"]
        k = m.get("id") or d.get("uuid")
        v = (m.get("model", "?"), u.get("input_tokens", 0), u.get("cache_creation_input_tokens", 0),
             u.get("output_tokens", 0), u.get("cache_read_input_tokens", 0))
        if k not in best or v[3] >= best[k][3]: best[k] = v
t = collections.defaultdict(lambda: [0, 0])
for model, i, cc, o, cr in best.values():
    t[model][0] += i + cc + o; t[model][1] += cr
print("; ".join(f"{m}: {a/1e3:.0f}k (+{c/1e6:.1f}M cached)" for m, (a, c) in sorted(t.items()) if m != "<synthetic>"))
PY
)
mkdir -p "$(dirname "$LOG")"
if [ ! -f "$LOG" ]; then
  cat > "$LOG" <<'HDR'
# Agent log

One row per finished task, written by `scripts/agent-log.sh` (see `docs/notes/AGENT_WORKFLOW.md` §7). Tokens are measured
from the session transcript and its subagents: input + cache creation + output per model, cache reads in parentheses.
`?` means not recorded, never guessed. The whole transcript of a session is counted, so a session that did several
tasks charges each row with everything up to that point: compare rows from separate sessions, or read the difference
between rows of one. Reviewed at each release (`scripts/agent-log.sh --summary <last tag>`): at most one process change,
then compare.

| date | task | mode | commits | tokens by model | session | gate first time | review found | fixes after push | user corrections | note |
|---|---|---|---|---|---|---|---|---|---|---|
HDR
fi
printf '| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |\n' "$(date +%F)" "$LABEL" "$MODE" "$COMMITS" "$TOK" \
  "$(basename "$SESSION" .jsonl | cut -c1-8)" "$GATE" "$REVIEW" "$FIXES" "$CORR" "${NOTE:-}" >> "$LOG"
tail -n 1 "$LOG"
