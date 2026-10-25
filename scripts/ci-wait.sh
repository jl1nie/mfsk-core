#!/usr/bin/env bash
# Wait for the CI runs of a commit and say, in one last line, whether they passed.
#
#   scripts/ci-wait.sh                          the commit at HEAD
#   scripts/ci-wait.sh <sha>                    another commit
#   scripts/ci-wait.sh --need 'Kotlin|Swift'    also require a SUCCEEDED job whose name matches
#                                               (an absent job is a failure, not a pass)
#   scripts/ci-wait.sh --timeout 1800           give up after this many seconds (default 3600)
#
# Run it in the background right after a push and do the work that does not depend on CI
# meanwhile. The LAST line is `CI PASS <sha8>` or `CI FAIL: <what>` (failed job names, a run that
# never appeared, a `--need` job that is missing or not green, a timeout), and the exit status
# is 0 only for PASS.
#
# What it does not know: path filters can leave a push with fewer runs than the full CI, and
# jobs a run skips are `skipped`, which counts as fine. `--need` is how a caller pins the jobs
# that matter for the change (the Kotlin and Swift jobs after a binding edit: they are the only
# place those sources are compiled).
set -uo pipefail

cd "$(git rev-parse --show-toplevel)"

SHA=""
NEED=""
TIMEOUT=3600
while [ $# -gt 0 ]; do
  case "$1" in
    --need) NEED="${2:-}"; shift 2 ;;
    --timeout) TIMEOUT="${2:-3600}"; shift 2 ;;
    -h | --help)
      sed -n '2,/^set -uo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
      exit 0
      ;;
    -*)
      echo "ci-wait.sh: unknown option '$1'" >&2
      exit 2
      ;;
    *) SHA="$1"; shift ;;
  esac
done
SHA="$(git rev-parse "${SHA:-HEAD}")" || { echo "CI FAIL: no such commit"; exit 2; }
short="${SHA:0:8}"
start=$SECONDS

# `gh run list --commit` returns the runs of that commit; give them a minute to appear.
runs_json() { gh run list --commit "$SHA" --limit 30 --json databaseId,name,status,conclusion 2>/dev/null; }

appear_deadline=$((SECONDS + 90))
while :; do
  json="$(runs_json)"
  count="$(jq 'length' <<<"${json:-[]}" 2>/dev/null || echo 0)"
  [ "${count:-0}" -gt 0 ] && break
  if [ "$SECONDS" -ge "$appear_deadline" ]; then
    echo "CI FAIL: no run appeared for $short within 90 s (path filter, or not pushed?)"
    exit 1
  fi
  sleep 10
done

while :; do
  json="$(runs_json)"
  pending="$(jq '[.[] | select(.status != "completed")] | length' <<<"$json")"
  [ "$pending" -eq 0 ] && break
  if [ $((SECONDS - start)) -ge "$TIMEOUT" ]; then
    echo "CI FAIL: timed out after ${TIMEOUT}s with $pending run(s) still going for $short"
    exit 1
  fi
  sleep 20
done

problems=()
while IFS=$'\t' read -r id name conclusion; do
  echo "run  $conclusion  $name"
  case "$conclusion" in
    success | skipped | neutral) ;;
    *) problems+=("run '$name' $conclusion") ;;
  esac
done < <(jq -r '.[] | [.databaseId, .name, .conclusion] | @tsv' <<<"$json")

# Jobs: print the ones that did not succeed, and check --need.
alljobs=""
for id in $(jq -r '.[].databaseId' <<<"$json"); do
  jobs="$(gh run view "$id" --json jobs --jq '.jobs[] | [.conclusion, .name] | @tsv' 2>/dev/null)"
  alljobs+="$jobs"$'\n'
done
while IFS=$'\t' read -r conclusion name; do
  [ -z "${name:-}" ] && continue
  case "$conclusion" in
    success | skipped | neutral) ;;
    *) echo "job  $conclusion  $name"; problems+=("job '$name' $conclusion") ;;
  esac
done <<<"$alljobs"

if [ -n "$NEED" ]; then
  matching="$(grep -E "$NEED" <<<"$alljobs" || true)"
  if [ -z "$matching" ]; then
    problems+=("no job matching /$NEED/ ran")
  else
    ok="$(grep -cE "^success"$'\t' <<<"$matching" || true)"
    total="$(grep -c . <<<"$matching" || true)"
    echo "need /$NEED/: $ok of $total job(s) succeeded"
    if [ "$ok" != "$total" ]; then
      problems+=("/$NEED/: $ok of $total job(s) succeeded")
    fi
  fi
fi

if [ "${#problems[@]}" -eq 0 ]; then
  echo "CI PASS $short"
  exit 0
fi
IFS=';'
echo "CI FAIL: ${problems[*]}"
exit 1
