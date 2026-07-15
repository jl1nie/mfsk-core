#!/usr/bin/env bash
# Install the project's git hooks into .git/hooks/.
# Run once after cloning:  bash scripts/install-hooks.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
HOOKS_DIR="$REPO_ROOT/.git/hooks"

install_hook() {
  local name="$1" target="$2"
  local hook_path="$HOOKS_DIR/$name"
  cat > "$hook_path" <<EOF
#!/usr/bin/env bash
exec "\$(git rev-parse --show-toplevel)/$target" "\$@"
EOF
  chmod +x "$hook_path"
  echo "  installed $name → $target"
}

echo "Installing git hooks for mfsk-core…"
install_hook pre-push scripts/pre-push-check.sh
echo "Done."
