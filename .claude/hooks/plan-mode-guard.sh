#!/usr/bin/env bash
# PreToolUse guard for ExitPlanMode (docs/notes/AGENT_WORKFLOW.md §2a).
# A plan declares its mode on a `Mode:` line, and carries that mode's required evidence:
#   Mode: port    names the upstream it ports (`Ported from` or a `lib/*.f90` / `lib/*.c` path)
#   Mode: design  says which test fails if a definition is wrong (`fails if`)
# The plan text comes from tool_input.plan; if the harness does not pass it, the newest
# ~/.claude/plans/*.md is read instead.
set -euo pipefail
input=$(cat)
plan=$(jq -r '.tool_input.plan // ""' <<<"$input")
if [[ -z "$plan" ]]; then
  newest=$(ls -t "$HOME"/.claude/plans/*.md 2>/dev/null | head -n1 || true)
  [[ -n "$newest" ]] && plan=$(cat "$newest")
fi
if [[ -n "${PLAN_GUARD_DEBUG:-}" ]]; then
  printf '%s\n' "$input" >>"$PLAN_GUARD_DEBUG"
fi

deny() {
  jq -n --arg r "$1" '{hookSpecificOutput:{hookEventName:"PreToolUse",permissionDecision:"deny",permissionDecisionReason:$r}}'
  exit 0
}

mode=$(grep -m1 -iE '^\**Mode:' <<<"$plan" | sed -E 's/^\**[Mm]ode:\**[[:space:]]*//' | tr 'A-Z' 'a-z' || true)
if [[ -z "$mode" ]]; then
  deny "AGENT_WORKFLOW.md §2a: the plan has no 'Mode:' line. Start it with 'Mode: port', 'Mode: design' or 'Mode: port + design' and add that mode's evidence."
fi
if [[ "$mode" == *port* ]] && ! grep -qiE 'Ported from|lib/[A-Za-z0-9_]+\.(f90|c)|wsjtx|upstream' <<<"$plan"; then
  deny "AGENT_WORKFLOW.md §2a: Mode: port needs the upstream it ports (file:line, or 'Ported from ...')."
fi
if [[ "$mode" == *design* ]] && ! grep -qi 'fails if' <<<"$plan"; then
  deny "AGENT_WORKFLOW.md §2a: Mode: design needs, per new public item, the test that 'fails if' its definition is wrong."
fi
if [[ "$mode" != *port* && "$mode" != *design* ]]; then
  deny "AGENT_WORKFLOW.md §2a: unknown mode '$mode' (use port, design, or port + design)."
fi
exit 0
