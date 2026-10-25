#!/usr/bin/env bash
# "Is it green?" in one command, with an answer that cannot be misread.
#
#   scripts/verify.sh                run every stage, stop at the first failure
#   scripts/verify.sh --keep-going   run every stage whatever fails
#   scripts/verify.sh --no-esp       skip the embedded `+esp` checks
#   scripts/verify.sh --quick        only the pre-push gate (what a push would run)
#
# Stages, in order:
#   pre-push   scripts/pre-push-check.sh (fmt, clippy -D clippy::perf, rustdoc, the feature
#              matrix, mfsk-ffi under desktop and mobile, the FT8 fixed-point floors)
#   gate       the merge gate: MFSK_REQUIRE_CORPUS=1 cargo test -p mfsk-core
#              --features full,internal-testing --release   (tiers A + B)
#   clean      the same merge gate on a clean checkout of HEAD (a throwaway `git worktree`, no
#              untracked or ignored files): what CI sees. COMMIT FIRST: it tests HEAD, not the
#              working tree. This is the stage that catches a test that passes here only because a
#              generated, git-ignored corpus (`embedded-poc/assets/ft4_sweep`, ...) is on this disk.
#   skimmer    cargo test -p skimmer-core
#   esp        cargo check --release in each embedded-poc crate (skipped, and said so, when the
#              `esp` toolchain is absent)
#
# The LAST line of output is `VERIFY PASS` or `VERIFY FAIL: <stage>[, <stage>]`, and the exit
# status is 0 only for PASS. Read that line, and nothing else, to know the result: run this
# directly (`run_in_background` with no `; echo` around it) so a completion notification's exit
# code is this script's. Per-stage logs are kept in $VERIFY_DIR (default target/verify) and the
# tail of a failing one is printed.
#
# Why `clean` exists (2026-10-25): two new tests read `embedded-poc/assets/ft4_sweep/*.wav`, which is
# in `.gitignore`. They passed here, in the reviewer's run and in pre-push, and failed in CI
# ("MFSK_REQUIRE_CORPUS is set but ft4_sweep is absent"), turning `main` red.
#
# Why the rest exists (2026-10-25): a background `bash pre-push-check.sh > log; echo "exit $?" >> log`
# reported "completed (exit code 0)" for the *wrapper* while the script had exited 101; the
# result was only found by reading the log. This script makes the verdict one line and the exit
# status honest.
set -uo pipefail

cd "$(git rev-parse --show-toplevel)"

KEEP_GOING=0
RUN_ESP=1
QUICK=0
for arg in "$@"; do
  case "$arg" in
    --keep-going) KEEP_GOING=1 ;;
    --no-esp) RUN_ESP=0 ;;
    --quick) QUICK=1 ;;
    -h | --help)
      sed -n '2,/^set -uo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
      exit 0
      ;;
    *)
      echo "verify.sh: unknown option '$arg'" >&2
      exit 2
      ;;
  esac
done

VERIFY_DIR="${VERIFY_DIR:-target/verify}"
mkdir -p "$VERIFY_DIR"
failed=()
n=0

# stage <name> <command...>: run it with its output in a log; print one verdict line.
stage() {
  local name="$1"
  shift
  n=$((n + 1))
  local log="$VERIFY_DIR/$(printf '%02d' "$n")-$name.log"
  local start=$SECONDS
  "$@" >"$log" 2>&1
  local status=$?
  local took=$((SECONDS - start))
  # cargo test prints one `test result:` line per binary; sum them.
  local counts
  counts=$(awk '/^test result:/ {p += $4; f += $6; b++} END {if (b) printf "%d passed, %d failed, %d binaries", p, f, b}' "$log")
  local failed_tests=0
  if [ -n "$counts" ] && echo "$counts" | grep -qv " 0 failed"; then
    failed_tests=1
  fi
  if [ "$status" -eq 0 ] && [ "$failed_tests" -eq 0 ]; then
    echo "PASS  $name (${took}s)${counts:+ — $counts}"
    return 0
  fi
  echo "FAIL  $name (${took}s, exit $status)${counts:+ — $counts}   log: $log"
  echo "  ---- last lines of $log ----"
  tail -n 15 "$log" | sed 's/^/  | /'
  failed+=("$name")
  return 1
}

# Stop at the first failure unless --keep-going.
run() {
  stage "$@" || { [ "$KEEP_GOING" = 1 ] || finish; }
}

finish() {
  if [ "${#failed[@]}" -eq 0 ]; then
    echo "VERIFY PASS"
    exit 0
  fi
  local IFS=,
  echo "VERIFY FAIL: ${failed[*]}" | sed 's/,/, /g'
  exit 1
}

# The merge gate on a clean checkout of HEAD, sharing this clone's target directory (third-party
# crates are not rebuilt; the library is, because its path differs).
clean_checkout() {
  local root dir
  root="$(pwd)"
  dir="$root/$VERIFY_DIR/clean"
  git worktree remove --force "$dir" >/dev/null 2>&1
  rm -rf "$dir"
  git worktree add --detach "$dir" HEAD >/dev/null || return 1
  (cd "$dir" && MFSK_REQUIRE_CORPUS=1 CARGO_TARGET_DIR="$root/target" \
    cargo test -p mfsk-core --features full,internal-testing --release)
  local status=$?
  git worktree remove --force "$dir" >/dev/null 2>&1
  return $status
}

esp_check() {
  local ok=0
  for crate in m5stack-cores3-app m5stack-s3 m5stack-s3-app m5stack-core2-app; do
    echo "== $crate"
    (cd "embedded-poc/$crate" && cargo check --release) || ok=1
  done
  return $ok
}

run pre-push bash scripts/pre-push-check.sh
if [ "$QUICK" = 0 ]; then
  run gate env MFSK_REQUIRE_CORPUS=1 cargo test -p mfsk-core --features full,internal-testing --release
  if [ -n "$(git status --porcelain --untracked-files=no)" ]; then
    echo "NOTE  clean: the working tree has uncommitted changes; this stage tests HEAD only"
  fi
  run clean clean_checkout
  run skimmer cargo test -p skimmer-core
  if [ "$RUN_ESP" = 1 ]; then
    if rustup toolchain list 2>/dev/null | grep -q '^esp'; then
      run esp esp_check
    else
      echo "SKIP  esp (no 'esp' toolchain: the embedded crates were not compiled)"
    fi
  else
    echo "SKIP  esp (--no-esp: the embedded crates were not compiled)"
  fi
fi
finish
