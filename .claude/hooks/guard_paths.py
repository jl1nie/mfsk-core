#!/usr/bin/env python3
"""PostToolUse check for Edit/Write on .rs files: no hardcoded machine paths.

CLAUDE.md "Test fixture paths": in-repo assets via `asset_path!` or CARGO_MANIFEST_DIR,
out-of-tree via `option_env!("WSJTX_SAMPLES_DIR")`. A string literal starting with
/home/ or /Users/ only works on the machine that wrote it. Exit 2 feeds the message
back to the agent; the edit itself has already happened, so it fixes it next.
"""

import json
import re
import sys

LITERAL = re.compile(r'"(/home/|/Users/)[^"]*"')


def main():
    try:
        payload = json.load(sys.stdin)
    except ValueError:
        return 0
    ti = payload.get("tool_input") or {}
    path = ti.get("file_path") or ""
    if not path.endswith(".rs"):
        return 0
    text = ti.get("content") or ti.get("new_string") or ""
    if not text and ti.get("edits"):
        text = "\n".join(e.get("new_string", "") for e in ti["edits"])
    hits = [m.group(0) for m in LITERAL.finditer(text)]
    if hits:
        print(f"{path}: hardcoded machine path {hits[0]}. Use asset_path! / "
              'concat!(env!("CARGO_MANIFEST_DIR"), "/../embedded-poc/assets/<f>") for in-repo '
              'assets, option_env!("WSJTX_SAMPLES_DIR") with a clean skip for out-of-tree ones '
              "(CLAUDE.md, Test fixture paths). /tmp is fine for diagnostic output.",
              file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
