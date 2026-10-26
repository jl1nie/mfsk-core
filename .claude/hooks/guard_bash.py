#!/usr/bin/env python3
"""PreToolUse guard for Bash: the CLAUDE.md "Rules that bite" that a regex can enforce.

Reads the hook payload on stdin. Prints a PreToolUse decision (deny / ask) as JSON,
or nothing (normal permission flow). Each rule cites where it is written down, so the
message tells the agent what to do instead, not only that it was stopped.

Run by hand:  echo '{"tool_input":{"command":"cargo publish"}}' | .claude/hooks/guard_bash.py
"""

import json
import re
import shlex
import sys

SEPARATORS = {"&&", "||", ";", "|", "&", "(", ")", "\n"}
# cargo test flags that take a value, so the value is not mistaken for a test filter.
CARGO_VALUE_FLAGS = {
    "-p", "--package", "--features", "-F", "--test", "--bin", "--example", "--bench",
    "--target", "--target-dir", "-j", "--jobs", "--profile", "--manifest-path",
    "--color", "--message-format", "--exclude", "-Z", "--config",
}


def segments(command):
    """Split a shell line into simple commands (good enough for a guard, not a parser)."""
    try:
        lex = shlex.shlex(command, posix=True, punctuation_chars=True)
        lex.whitespace_split = True
        tokens = list(lex)
    except ValueError:
        tokens = command.split()
    seg = []
    for t in tokens:
        if t in SEPARATORS:
            if seg:
                yield seg
            seg = []
        else:
            seg.append(t)
    if seg:
        yield seg


def strip_env(seg):
    """Drop leading VAR=value assignments."""
    i = 0
    while i < len(seg) and re.match(r"^[A-Za-z_][A-Za-z0-9_]*=", seg[i]):
        i += 1
    return seg[i:]


def is_git(seg, sub):
    """git [-C dir] [-c k=v] <sub> ... -> args after <sub>, else None."""
    if not seg or seg[0] != "git":
        return None
    i = 1
    while i < len(seg) and seg[i] in ("-C", "-c"):
        i += 2
    if i < len(seg) and seg[i] == sub:
        return seg[i + 1:]
    return None


def check(seg):
    seg = strip_env(seg)
    if not seg:
        return None

    # CLAUDE.md "Releases": publishing goes through release.yml on a tag push.
    if seg[:2] == ["cargo", "publish"]:
        return ("deny", "Never `cargo publish` locally (CLAUDE.md, Releases). Push a `vX.Y.Z` "
                "tag and let release.yml publish after CI; see docs/notes/RELEASE_PROCEDURE.md.")

    args = is_git(seg, "add")
    if args is not None and any(a in ("-A", "--all", ".", ":/", "-u", "--update") for a in args):
        return ("deny", "Stage paths by name, not `git add -A` / `.` / `-u`: the user edits the "
                "tree during a session and a blanket add sweeps their work into your commit.")

    args = is_git(seg, "commit")
    if args is not None and any(re.match(r"^-[a-zA-Z]*a[a-zA-Z]*$", a) or a == "--all" for a in args):
        return ("deny", "`git commit -a` stages every tracked change, including the user's. "
                "`git add <paths>` first, then commit without -a.")

    args = is_git(seg, "push")
    if args is not None:
        if any(a in ("-f", "--force") or a.startswith("--force-with-lease") for a in args):
            return ("ask", "Force push: confirm with the user first.")
        if any(re.match(r"^(refs/tags/)?(skimmer-)?v\d", a) or a == "--tags" for a in args):
            return ("ask", "Pushing a release tag triggers release.yml (crates.io publish is "
                    "irreversible). Confirm scripts/release-status.sh is clean and the tier-C "
                    "sweeps it names have run (docs/notes/RELEASE_PROCEDURE.md).")

    # CLAUDE.md "Local checks and tests": never a blanket --ignored (escalates into tier C).
    if seg[:2] == ["cargo", "test"] and any(a in ("--ignored", "--include-ignored") for a in seg):
        before = seg[2:seg.index("--")] if "--" in seg else seg[2:]
        after = seg[seg.index("--") + 1:] if "--" in seg else []
        scoped = any(a == "--test" or a.startswith("--test=") for a in before)
        i = 0
        while i < len(before):
            a = before[i]
            if a in CARGO_VALUE_FLAGS:
                i += 2
                continue
            if not a.startswith("-"):
                scoped = True  # a test-name filter
            i += 1
        if any(not a.startswith("-") for a in after):
            scoped = True  # filter after `--`
        if not scoped:
            return ("deny", "Blanket `--ignored` escalates into tier C (hours, ~17 GB corpora). "
                    "Scope it: `--test <binary>` and/or a test-name filter. Tier C runs via "
                    "scripts/run-sensitivity-sweeps.sh [modes] (docs/notes/TESTING_GUIDE.md).")

    # embedded-poc/CLAUDE.md: logs only through flash-monitor.sh.
    if seg[:2] == ["espflash", "monitor"]:
        return ("deny", "A second `espflash monitor` resets S3 USB-OTG boards into DOWNLOAD mode. "
                "Capture with embedded-poc/scripts/flash-monitor.sh (docs/notes/EMBEDDED_WORKFLOW.md).")
    if seg[0] in ("cat", "head", "tail", "screen", "minicom") and any(
            re.match(r"^/dev/(tty(ACM|USB)|cu\.usbmodem)", a) for a in seg[1:]):
        return ("deny", "Don't read the serial port directly; use "
                "embedded-poc/scripts/flash-monitor.sh (docs/notes/EMBEDDED_WORKFLOW.md).")
    return None


def main():
    try:
        payload = json.load(sys.stdin)
    except ValueError:
        return 0
    command = (payload.get("tool_input") or {}).get("command") or ""
    verdict = None
    for seg in segments(command):
        v = check(seg)
        if v and (verdict is None or v[0] == "deny"):
            verdict = v
        if verdict and verdict[0] == "deny":
            break
    if verdict:
        json.dump({"hookSpecificOutput": {
            "hookEventName": "PreToolUse",
            "permissionDecision": verdict[0],
            "permissionDecisionReason": verdict[1],
        }}, sys.stdout)
    return 0


if __name__ == "__main__":
    sys.exit(main())
