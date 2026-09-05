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

# Matching the version is not the same as being current. The top
# section can name the right version and still predate everything
# merged since it was written — 77 commits, on 2026-08-23, when this
# check was added because the first version of this script did not
# catch it either.
cl_commit=$(git log -1 --format=%H -- CHANGELOG.md)
cl_date=$(git log -1 --format=%cs -- CHANGELOG.md)
cl_behind=$(git rev-list --count "$cl_commit..HEAD" -- . ':(exclude)CHANGELOG.md' 2>/dev/null || echo 0)
echo "  last edited: $cl_date"
if (( cl_behind == 0 )); then
    ok "current — nothing merged since it was last edited"
else
    warn "$cl_behind commits have landed since. Unreleased work is unrecorded."
    warn "   Embedded work belongs here too: 0.10.0's own heading names the"
    warn "   CoreS3 receiver, and embedded-poc is where most of it lives."
    git log --format='       %h %s' "$cl_commit..HEAD" -- . ':(exclude)CHANGELOG.md' | head -6
    (( cl_behind > 6 )) && echo "       … and $(( cl_behind - 6 )) more"
fi

# ── Cadence ──────────────────────────────────────────────────────────
bold "== cadence =="
if [[ -n "$last_tag" ]]; then
    tag_date=$(git log -1 --format=%cs "$last_tag")
    # Epoch straight from git rather than re-parsing `$tag_date` with
    # `date -d`: that flag is GNU-only and this repo's development
    # machine is macOS, where BSD `date` rejects it and the whole
    # cadence section died in an arithmetic error (2026-08-24). `%ct`
    # is the same commit's date as `%cs` above, already in seconds, so
    # nothing has to parse a date string at all.
    tag_epoch=$(git log -1 --format=%ct "$last_tag")
    days=$(( ( $(date +%s) - tag_epoch ) / 86400 ))
    n=$(git rev-list --count "$last_tag..HEAD" 2>/dev/null || echo '?')
    echo "  $last_tag was $days days ago ($tag_date), $n commits since"
    # Biweekly by convention; see CLAUDE.md "Release cadence". 13-14
    # days is the *target*, not an overrun — the old >=13 "past
    # maximum" wording flagged an on-time release as late, from
    # averaging in two escape-hatch same-week patches that dragged the
    # historical mean down to ~7 (corrected 2026-09-06, see CLAUDE.md's
    # own note on this).
    if (( days > 14 )); then
        warn "past the 2-week target — overdue"
    elif (( days >= 13 )); then
        ok "at the 2-week target — good time to cut"
    elif (( days >= 7 )); then
        ok "inside the window, before the 2-week target"
    else
        ok "recent — cutting now would be early unless the escape hatch applies"
    fi
fi

# ── Which sensitivity sweeps are actually needed ─────────────────────

# True when `$1` changes nothing but comments and blank lines under
# `$2`.
#
# A tier-C sweep is tens of minutes to hours per protocol, so a
# documentation pass that happens to touch a protocol directory must
# not read the same as a decoder change. #323 classified the crate's
# 12 kHz literals and added a doc comment to `ft8/params.rs`,
# `jt9/softsym.rs`, `wspr/baseband.rs` and `msk144/spd.rs` in one
# commit — which showed up here as four protocols "whose own source
# changed", against a diff that adds and removes no executable line.
#
# Rust line comments only. A `/* … */` body reads as code and the
# commit is reported as a code change: this errs toward re-sweeping
# something that did not need it, never toward skipping something that
# did, which is the only direction that is safe to be wrong in.
is_prose_only() {
    local sha="$1"; shift
    # A merge commit shows no diff under plain `git show`, which would
    # read as "no code changed". Treat it as code.
    [[ -n "$(git rev-list --parents -n1 "$sha" | cut -d' ' -f3-)" ]] && return 1

    # The changed lines, stripped of their +/- and leading space.
    #
    # Collected into a variable rather than piped straight into the
    # test below, because the obvious `… | grep -qEv` is a race under
    # this script's own `set -o pipefail`: `grep -q` exits on its first
    # match, `git show` and `sed` upstream then die of SIGPIPE (141),
    # and pipefail hands that 141 back as the pipeline's status — which
    # the `!` turns into "prose only". Whether the kill lands before
    # the upstream stages finish writing is pure timing, so the same
    # commit classified either way run to run: 84c3159 (367 new lines
    # in fst4/ddc.rs) came back "[comments only]" on 34 of 40 runs on
    # 2026-08-27, and the here-string version on 0 of 40. That is the
    # unsafe direction — a decoder change read as a comment skips a
    # sweep that was needed — and it is exactly what the contract
    # above promises cannot happen. A here-string has no upstream
    # process to signal.
    local body
    body=$(git show --format='' --unified=0 "$sha" -- "$@" \
        | grep -E '^[+-]' \
        | grep -Ev '^(\+\+\+|---)' \
        | sed -E 's/^[+-][[:space:]]*//')
    ! grep -qEv '^(//.*)?$' <<<"$body"
}

bold "== tier-C sweeps =="
baseline=docs/notes/sweep-baseline.json
file_date=$(git log -1 --format=%cs -- "$baseline")

# Per-protocol baseline dates out of the JSON's `_meta` block, so a
# partial re-sweep ("run-sensitivity-sweeps.sh ft4 fst4") only silences
# the protocols it actually measured. Before `_meta` existed there was
# one date for the whole file — the file's own last-commit date — which
# meant refreshing two protocols marked all nine as current, and
# conversely a docs commit touching the file aged nothing but still
# reset the comparison. Falls back to the file date when `_meta` is
# absent or python3 isn't available, which is the old behaviour.
declare -A base_dates=()
if command -v python3 >/dev/null 2>&1; then
    while IFS='=' read -r k v; do
        [[ -n "$k" ]] && base_dates["$k"]="$v"
    done < <(python3 - "$baseline" <<'PY' 2>/dev/null
import json, sys
try:
    meta = json.load(open(sys.argv[1])).get("_meta", {})
except (OSError, ValueError):
    sys.exit(0)
for proto, m in meta.get("protocols", {}).items():
    if m.get("date"):
        print(f"{proto}={m['date']}")
PY
    )
fi

if (( ${#base_dates[@]} == 0 )); then
    echo "  baseline last updated: $file_date ($baseline, no per-protocol _meta)"
else
    oldest=$(printf '%s\n' "${base_dates[@]}" | sort | head -1)
    newest=$(printf '%s\n' "${base_dates[@]}" | sort | tail -1)
    if [[ "$oldest" == "$newest" ]]; then
        echo "  baseline last updated: $oldest ($baseline, all ${#base_dates[@]} protocols)"
    else
        echo "  baseline last updated: $oldest .. $newest ($baseline, per protocol)"
    fi
fi
echo "  protocols whose own source changed since their own baseline:"
needed=()
shared=0
for proto in ft8 ft4 fst4 wspr jt65 jt9 q65 msk144; do
    d="mfsk-core/src/$proto"
    [[ -d "$d" ]] || continue
    since="${base_dates[$proto]:-$file_date}"
    # Subjects, not just a count. A count sends the reader off to look
    # them up, which is the hand-reconstruction this script exists to
    # remove — and a clippy sweep counts the same as a decoder change.
    mapfile -t subjects < <(git log --since="$since" --format='%h %s' -- "$d")
    (( ${#subjects[@]} > 0 )) || continue
    code=0
    rendered=()
    for line in "${subjects[@]}"; do
        if is_prose_only "${line%% *}" "$d"; then
            rendered+=("               ${line:0:72}  [comments only]")
        else
            rendered+=("               ${line:0:88}")
            code=$(( code + 1 ))
        fi
    done
    printf '    %-8s since %s: %d commit(s), %d touching code\n' \
        "$proto" "$since" "${#subjects[@]}" "$code"
    printf '%s\n' "${rendered[@]}"
    # Prose-only commits are printed, not hidden — "nothing to sweep"
    # should be visibly derived rather than silently assumed.
    (( code > 0 )) && needed+=("$proto")
done
if (( ${#needed[@]} == 0 )); then
    ok "none — the recorded sweep still stands"
else
    echo
    echo "    scripts/run-sensitivity-sweeps.sh ${needed[*]}"
fi

# Shared code is the part a protocol list cannot decide for you.
while IFS= read -r sha; do
    [[ -n "$sha" ]] || continue
    is_prose_only "$sha" mfsk-core/src/engine mfsk-core/src/dsp && continue
    shared=$(( shared + 1 ))
done < <(git log --since="${oldest:-$file_date}" --format=%h -- mfsk-core/src/engine mfsk-core/src/dsp 2>/dev/null)
if (( shared > 0 )); then
    echo
    warn "$shared commits touched mfsk-core/src/{engine,dsp} — shared code."
    warn "   Sharing a module name is not sharing a code path. FT8 reaches"
    warn "   sync and spectrogram through its own decode_block copies, so an"
    warn "   engine::sync change cannot move an FT8 curve (issue #280, learned"
    warn "   the expensive way). Check the callers before adding a protocol."
fi

if [[ -d target/sweep-csv ]]; then
    fresh=$(find target/sweep-csv -name '*.csv' -newermt "${oldest:-$file_date}" 2>/dev/null | wc -l)
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
