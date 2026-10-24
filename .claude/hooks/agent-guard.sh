#!/usr/bin/env bash
# PreToolUse guard for the Agent tool (docs/notes/AGENT_WORKFLOW.md §3).
# Refuses the two ways an implementation silently runs on the main (Opus) model:
# a `fork` (inherits the main model) and a `model` override on `runner`.
set -euo pipefail
input=$(cat)
type=$(jq -r '.tool_input.subagent_type // ""' <<<"$input")
model=$(jq -r '.tool_input.model // ""' <<<"$input")

deny() {
  jq -n --arg r "$1" '{hookSpecificOutput:{hookEventName:"PreToolUse",permissionDecision:"deny",permissionDecisionReason:$r}}'
  exit 0
}

if [[ "$type" == "fork" ]]; then
  deny "AGENT_WORKFLOW.md §3: no fork (it inherits the main model). Do the work in the main loop, switch /model, or use runner."
fi
if [[ "$type" == "runner" && -n "$model" ]]; then
  deny "AGENT_WORKFLOW.md §3: runner's model comes from .claude/agents/runner.md; drop the model override."
fi
exit 0
