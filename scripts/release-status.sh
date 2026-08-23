#!/usr/bin/env bash
# Print the release state, computed from the repository.
#
# WHY THIS EXISTS
#
# Release preparation kept going wrong the same way: the state was
# reconstructed by hand each time, from memory and inference, and the
# facts involved are all mechanically derivable in seconds. Three
# mistakes in one session on 2026-08-23, all of this shape:
#
#   - "the tier-C sweeps still need running" — they had been run, and
#     the note recording that said not to re-run them unless something
#     protocol-relevant had changed. The condition was never evaluated.
#   - "#282 changed JT65/JT9/Q65, so those need re-sweeping" — those
#     commits predate the sweep. A date comparison would have said so.
#   - `0.10.0` sat un-tagged, with its CHANGELOG section written and
#     the workspace version bumped, through an entire conversation
#     about preparing a release.
#
# `CLAUDE.md` documents the *procedure*. Nothing computed the *state*,
# so every attempt re-derived it, and re-derivation is where the errors
# came from. Run this first.
#
#   scripts/release-status.sh
#
# Read-only. Exits 0 always — it reports, it does not gate.
set -uo pipefail
cd "$(dirname "$0")/.."

bold() { printf '\033[1m%s\033[0m\n' "$*"; }
warn() { printf '  \033[33m!! %s\033[0m\n' "$*"; }
ok()   { printf '  \033[32mok\033[0m %s\n' "$*"; }

# ── Version vs tags ──────────────────────────────────────────────────
bold "== version =="
version=$(grep -m1 '^version = ' Cargo.toml | sed 's/.*"\(.*\)".*/\1/')
last_tag=$(git tag -l 'v*' --sort=-creatordate | head -1)
echo "  Cargo.toml : $version"
echo "  last tag   : ${last_tag:-<none>}"
if [[ "v$version" == "$last_tag" ]]; then
    ok "tagged"
else
    warn "v$version is NOT tagged — a release may be half-finished."
    warn "   CD runs on the tag push, so nothing has reached crates.io."
fi

changelog_top=$(grep -m1 '^## ' CHANGELOG.md | sed 's/^## //')
echo "  CHANGELOG  : ${changelog_top:0:60}"
case "$changelog_top" in
    "$version"*) ok "CHANGELOG's top section matches the version" ;;
    *) warn "CHANGELOG's top section does not start with $version" ;;
esac

# ── Cadence ──────────────────────────────────────────────────────────
bold "== cadence =="
if [[ -n "$last_tag" ]]; then
    tag_date=$(git log -1 --format=%cs "$last_tag")
    days=$(( ( $(date +%s) - $(date -d "$tag_date" +%s) ) / 86400 ))
    n=$(git rev-list --count "$last_tag..HEAD" 2>/dev/null || echo '?')
    echo "  $last_tag was $days days ago ($tag_date), $n commits since"
    # Biweekly by convention; see CLAUDE.md "Release cadence".
    if (( days >= 13 )); then
        warn "past the 13-day maximum of the biweekly cadence"
    elif (( days >= 7 )); then
        ok "inside the window (average is ~7 days)"
    else
        ok "recent — cutting now would be early unless the escape hatch applies"
    fi
fi

# ── Which sensitivity sweeps are actually needed ─────────────────────
bold "== tier-C sweeps =="
baseline=docs/notes/sweep-baseline.json
base_date=$(git log -1 --format=%cs -- "$baseline")
echo "  baseline last updated: $base_date ($baseline)"
echo "  protocols whose own source changed since then:"
needed=()
for proto in ft8 ft4 fst4 wspr jt65 jt9 q65 msk144; do
    d="mfsk-core/src/$proto"
    [[ -d "$d" ]] || continue
    # Subjects, not just a count. A count sends the reader off to look
    # them up, which is the hand-reconstruction this script exists to
    # remove — and a clippy sweep counts the same as a decoder change.
    mapfile -t subjects < <(git log --since="$base_date" --format='%h %s' -- "$d")
    if (( ${#subjects[@]} > 0 )); then
        printf '    %-8s %d commit(s)\n' "$proto" "${#subjects[@]}"
        for line in "${subjects[@]}"; do
            printf '               %s\n' "${line:0:88}"
        done
        needed+=("$proto")
    fi
done
if (( ${#needed[@]} == 0 )); then
    ok "none — the recorded sweep still stands"
else
    echo
    echo "    scripts/run-sensitivity-sweeps.sh ${needed[*]}"
fi

# Shared code is the part a protocol list cannot decide for you.
shared=$(git log --since="$base_date" --oneline -- mfsk-core/src/engine mfsk-core/src/dsp 2>/dev/null | wc -l)
if (( shared > 0 )); then
    echo
    warn "$shared commits touched mfsk-core/src/{engine,dsp} — shared code."
    warn "   Sharing a module name is not sharing a code path. FT8 reaches"
    warn "   sync and spectrogram through its own decode_block copies, so an"
    warn "   engine::sync change cannot move an FT8 curve (issue #280, learned"
    warn "   the expensive way). Check the callers before adding a protocol."
fi

if [[ -d target/sweep-csv ]]; then
    fresh=$(find target/sweep-csv -name '*.csv' -newermt "$base_date" 2>/dev/null | wc -l)
    echo
    echo "  target/sweep-csv: $(ls target/sweep-csv/*.csv 2>/dev/null | wc -l) CSVs, $fresh newer than the baseline"
    echo "  compare with: python3 scripts/sweep-regression-check.py target/sweep-csv/*.csv"
fi

# ── Merge gate ───────────────────────────────────────────────────────
bold "== working tree =="
dirty=$(git status --porcelain | wc -l)
(( dirty == 0 )) && ok "clean" || warn "$dirty uncommitted paths"
branch=$(git rev-parse --abbrev-ref HEAD)
echo "  branch: $branch"
