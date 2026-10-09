#!/usr/bin/env bash
# What changed in WSJT-X since the version we ported, and which of our files cite it.
#
#   scripts/upstream-diff.sh                       # BASE..newest v* tag on `upstream`
#   scripts/upstream-diff.sh v3.2.0-rc1 v3.3.0-beta1
#   scripts/upstream-diff.sh --no-fetch --all      # offline; also list uncited lib/ files
#   WSJTX_DIR=/path/to/WSJT-X scripts/upstream-diff.sh
#
# WHY THIS EXISTS
#
# The 2026-10-10 audit of v3.2.0-rc1 -> v3.3.0-beta1 (issue #642) was done by
# hand, and two things nearly went wrong that a script can state mechanically:
#
#   - rc1 is NOT an ancestor of beta1 (merge-base 0b32aba69, 2026-09-18). rc1 was
#     cut from the development line and tagged a week later; development went on.
#     `git log BASE..TARGET` across such a fork counts commits that mean nothing
#     ("277 commits"). The comparison that is correct is the TREE diff, which is
#     what this script prints. It says so when the fork applies.
#   - `origin` of the usual clone (saitohirga/WSJT-X) stopped in 2024-03; the live
#     repository is https://github.com/WSJTX/wsjtx (remote `upstream`), and its
#     `master` is the 3.0.x line, not the development tip. Following `master` or
#     `origin` reports "nothing new" while the algorithms move on release/3.x.
#
# The mapping from upstream file to our code is not guessed: every algorithm file
# in mfsk-core/src cites the lib/*.f90 / lib/*.c it ports (CLAUDE.md, "Cite the
# upstream"), so the script greps those citations. A changed upstream file with no
# citation is either not ported (SuperFox, ft8var, GUI) or a citation is missing.
#
# What it cannot do is classify a change. Whether a hunk alters decode results or is
# a refactor (the 3.3 PCM-engine move was ~80% refactor) needs reading the diff;
# the per-file `git diff` command is printed for exactly that.
#
# Read-only apart from `git fetch` into the WSJT-X clone. Exits 0 always.
set -uo pipefail
cd "$(dirname "$0")/.."
REPO=$PWD

# The version the port currently tracks. Bump it when a port PR for a newer
# upstream merges, together with the Issue that recorded the audit.
DEFAULT_BASE=v3.2.0-rc1
REMOTE=upstream

fetch=1; show_all=0; base=""; target=""
for a in "$@"; do
    case $a in
        --no-fetch) fetch=0 ;;
        --all)      show_all=1 ;;
        -h|--help)  sed -n '2,/^set -uo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) if [[ -z $base ]]; then base=$a; elif [[ -z $target ]]; then target=$a;
           else echo "too many arguments" >&2; exit 2; fi ;;
    esac
done
base=${base:-$DEFAULT_BASE}

bold() { printf '\033[1m%s\033[0m\n' "$*"; }
warn() { printf '  \033[33m!! %s\033[0m\n' "$*"; }
ok()   { printf '  \033[32mok\033[0m %s\n' "$*"; }

W=${WSJTX_DIR:-$REPO/../WSJT-X}
if [[ ! -d $W/.git ]]; then
    echo "no WSJT-X clone at $W (set WSJTX_DIR)" >&2; exit 0
fi
g() { git -C "$W" "$@"; }

bold "== upstream =="
if ! g remote get-url "$REMOTE" >/dev/null 2>&1; then
    warn "remote '$REMOTE' missing in $W — add https://github.com/WSJTX/wsjtx.git"
    exit 0
fi
echo "  clone  : $W"
echo "  remote : $REMOTE = $(g remote get-url $REMOTE)"
if (( fetch )); then
    g fetch --tags --quiet "$REMOTE" 2>&1 | sed 's/^/  fetch: /' || true
fi
origin_url=$(g remote get-url origin 2>/dev/null || true)
if [[ -n $origin_url ]]; then
    od=$(g log -1 --format=%cs "origin/HEAD" 2>/dev/null || true)
    [[ -n $od ]] && echo "  origin : $origin_url (last commit ${od}) — not followed"
fi

# Default target: the newest v* tag by version order, prereleases included.
if [[ -z $target ]]; then
    target=$(g tag -l 'v*' --sort=-version:refname | head -1)
    # `sort -version` ranks 3.3.0-beta1 below 3.3.0 and above 3.2.x, which is what we want,
    # but ranks rc/beta of an older minor above nothing else; fall back to date if empty.
    [[ -z $target ]] && target=$(g tag -l 'v*' --sort=-creatordate | head -1)
fi
for r in "$base" "$target"; do
    g rev-parse -q --verify "$r^{commit}" >/dev/null || { warn "unknown ref: $r"; exit 0; }
done

bold "== versions (by date) =="
g tag -l 'v*' --sort=-creatordate --format='%(creatordate:short) %(refname:short)' | head -6 | sed 's/^/  /'
echo "  branches on $REMOTE (latest commit):"
g for-each-ref --sort=-committerdate --count=6 \
    --format='    %(committerdate:short) %(refname:short)' "refs/remotes/$REMOTE" \
    | grep -v 'feat-\|docs/\|bug-report'
echo
echo "  base   : $base  ($(g log -1 --format=%cs "$base^{commit}") $(g rev-parse --short "$base^{commit}"))"
echo "  target : $target  ($(g log -1 --format=%cs "$target^{commit}") $(g rev-parse --short "$target^{commit}"))"
newer=$(g tag -l 'v*' --sort=-creatordate --format='%(refname:short)' | awk -v b="$base" '$0==b{exit} {print}')
[[ -n $newer ]] && echo "  tags newer than base: $(echo $newer)"

bold "== topology =="
mb=$(g merge-base "$base" "$target" 2>/dev/null || true)
if [[ -z $mb ]]; then
    warn "no common ancestor"
elif [[ $mb == "$(g rev-parse "$base^{commit}")" ]]; then
    ok "base is an ancestor of target; $(g rev-list --count "$base..$target") commits"
else
    warn "base is NOT an ancestor of target (fork at $(g rev-parse --short "$mb"), $(g log -1 --format=%cs "$mb"))"
    warn "  base side: $(g rev-list --count "$mb..$base") commits, target side: $(g rev-list --count "$mb..$target")"
    warn "  commit counts and 'git log base..target' mean nothing here; the TREE diff below is the comparison"
fi

# ── citations in our source: basename -> files citing it ───────────────
cites=$(mktemp); trap 'rm -f "$cites"' EXIT
grep -rnoE '[A-Za-z0-9_]+\.(f90|F90|c|h|cpp)\b' mfsk-core/src mfsk-ffi/src embedded-poc/embedded-shared 2>/dev/null \
    | awk -F: '{ print $3 "\t" $1 }' | sort -u > "$cites"
cited_by() { awk -F'\t' -v f="$1" '$1==f{print $2}' "$cites" | sort -u; }

# Tree diff, whitespace- and line-ending-insensitive (as in #435). Only lib/ and
# map65/ hold the algorithms; the GUI, tests, docs and CI are not ported.
mapfile -t rows < <(g diff -w --ignore-cr-at-eol --numstat "$base" "$target" -- lib map65/libm65 \
    | grep -vE '\.(ts|md|txt|png|py|cmake|json)\t?$' | grep -vE '\.ts$|/tests?/|\.md$|\.txt$|\.png$')

bold "== changed upstream files that mfsk-core cites ($base -> $target) =="
n_cited=0; n_unc=0; unc=()
for row in "${rows[@]}"; do
    IFS=$'\t' read -r add del path <<<"$row"
    [[ $add == - ]] && continue
    bn=${path##*/}
    users=$(cited_by "$bn")
    if [[ -n $users ]]; then
        n_cited=$((n_cited+1))
        printf '  %-44s +%-5s -%-5s\n' "$path" "$add" "$del"
        nu=$(echo "$users" | wc -l)
        echo "$users" | head -3 | sed 's/^/        cited by /'
        (( nu > 3 )) && echo "        … and $((nu-3)) more"
        printf '        read: git -C %s diff -w --ignore-cr-at-eol %s %s -- %s\n' "$W" "$base" "$target" "$path"
    else
        n_unc=$((n_unc+1)); unc+=("$add	$del	$path")
    fi
done
(( n_cited == 0 )) && ok "no cited upstream file changed"

bold "== changed upstream files not cited anywhere (not ported, or a citation is missing) =="
echo "  $n_unc files"
if (( show_all )); then
    printf '%s\n' "${unc[@]}" | sort -t$'\t' -k1 -nr | awk -F'\t' '{printf "  %-44s +%-5s -%-5s\n",$3,$1,$2}'
else
    printf '%s\n' "${unc[@]}" | sort -t$'\t' -k1 -nr | head -12 | awk -F'\t' '{printf "  %-44s +%-5s -%-5s\n",$3,$1,$2}'
    (( n_unc > 12 )) && echo "  … ($((n_unc-12)) more; --all lists them)"
fi

bold "== where to record the result =="
echo "  Class each change A (alters results) / B (real bug fix) / C (refactor, plumbing); only A and B"
echo "  need a port. Record it in an Issue (see #435, #642); bump DEFAULT_BASE in this script when"
echo "  the port PRs for $target have merged."
exit 0
