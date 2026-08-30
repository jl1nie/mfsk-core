#!/usr/bin/env bash
# Point git at this repo's hooks.  Run once after cloning:
#   bash scripts/install-hooks.sh
#
# Both hooks live in `.githooks/` and are enabled by a single config
# setting, so this script is a convenience wrapper around one line:
#
#   git config core.hooksPath .githooks
#
# It used to *copy* a pre-push shim into `.git/hooks/` instead, which
# could never fire: `core.hooksPath` — set by this same repo's own
# setup instructions, in README.md and CONTRIBUTING.md — makes git read
# hooks from that directory *instead of* `$GIT_DIR/hooks`, never both.
# So following both instructions silently disabled the feature matrix,
# and nothing said so. Verified 2026-08-30; see `.githooks/pre-push`.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO_ROOT"

git config core.hooksPath .githooks
echo "core.hooksPath -> .githooks"
for h in .githooks/*; do
  [ -x "$h" ] && echo "  enabled $(basename "$h")"
done

# A shim left behind by the old version of this script is now dead
# weight — git will not run it — but leaving it in place invites the
# next reader to believe the gate is installed twice.
if [ -e .git/hooks/pre-push ]; then
  echo
  echo "note: .git/hooks/pre-push exists and is now SHADOWED by core.hooksPath."
  echo "      It is a leftover from the previous install method and never runs."
  echo "      Remove it with:  rm .git/hooks/pre-push"
fi
