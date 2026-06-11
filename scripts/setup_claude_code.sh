#!/usr/bin/env bash
# setup_claude_code.sh — Install Claude Code CLI persistently on the Amiga brain.
#
# Everything is installed under ~/ which lives on the NVMe and survives the
# overlay-filesystem reboot wipe.  Node.js is managed via nvm (~/.nvm).
#
# Usage (run once on the brain, then open a new SSH session):
#   bash scripts/setup_claude_code.sh
#
# After installation:
#   cd ~/auto_navigation_vidalia && claude

set -euo pipefail

NVM_VERSION="v0.40.1"
NODE_ALIAS="lts/*"

info()  { echo "[claude-setup] $*"; }
ok()    { echo "[claude-setup] ✓ $*"; }
warn()  { echo "[claude-setup] ⚠ $*"; }

info "Starting on $(hostname)  arch=$(uname -m)  os=$(. /etc/os-release && echo "$PRETTY_NAME")"

# ---------------------------------------------------------------------------
# 1. nvm
# ---------------------------------------------------------------------------
export NVM_DIR="$HOME/.nvm"

if [ -s "$NVM_DIR/nvm.sh" ]; then
    ok "nvm already present at ~/.nvm"
    source "$NVM_DIR/nvm.sh"
else
    info "Downloading nvm ${NVM_VERSION} …"
    curl -fsSL "https://raw.githubusercontent.com/nvm-sh/nvm/${NVM_VERSION}/install.sh" | bash
    source "$NVM_DIR/nvm.sh"
    ok "nvm installed"
fi

# Ensure nvm init is in ~/.bashrc (nvm installer does this, but be safe)
if ! grep -q 'NVM_DIR' "$HOME/.bashrc" 2>/dev/null; then
    cat >> "$HOME/.bashrc" <<'EOF'

# nvm (Node Version Manager) — added by setup_claude_code.sh
export NVM_DIR="$HOME/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && source "$NVM_DIR/nvm.sh"
EOF
    info "Added nvm init to ~/.bashrc"
fi

# ---------------------------------------------------------------------------
# 2. Node.js LTS
# ---------------------------------------------------------------------------
CURRENT_NODE=$(node --version 2>/dev/null || echo "none")
if [[ "$CURRENT_NODE" =~ ^v(18|20|22|24|26) ]]; then
    ok "Node.js $CURRENT_NODE is already suitable"
else
    info "Installing Node.js LTS (current: $CURRENT_NODE) …"
    nvm install "$NODE_ALIAS"
    nvm alias default "$NODE_ALIAS"
    nvm use default
    ok "Node.js $(node --version) installed"
fi

info "Node: $(node --version)   npm: $(npm --version)"

# ---------------------------------------------------------------------------
# 3. Claude Code CLI
# ---------------------------------------------------------------------------
if command -v claude &>/dev/null; then
    INSTALLED=$(claude --version 2>/dev/null || echo "unknown")
    info "Claude Code already installed ($INSTALLED) — updating …"
    npm update -g @anthropic-ai/claude-code
else
    info "Installing @anthropic-ai/claude-code …"
    npm install -g @anthropic-ai/claude-code
fi

ok "Claude Code $(claude --version 2>/dev/null)"

# ---------------------------------------------------------------------------
# 4. ANTHROPIC_API_KEY
# ---------------------------------------------------------------------------
echo ""
if [ -n "${ANTHROPIC_API_KEY:-}" ]; then
    ok "ANTHROPIC_API_KEY is already set in the environment"
else
    warn "ANTHROPIC_API_KEY is not set.  Add it permanently:"
    echo ""
    echo "    echo 'export ANTHROPIC_API_KEY=\"sk-ant-...\"' >> ~/.bashrc"
    echo "    source ~/.bashrc"
    echo ""
    read -rp "[claude-setup] Enter your API key now to add it to ~/.bashrc (or press Enter to skip): " apikey
    if [ -n "$apikey" ]; then
        echo "export ANTHROPIC_API_KEY=\"$apikey\"" >> "$HOME/.bashrc"
        export ANTHROPIC_API_KEY="$apikey"
        ok "API key saved to ~/.bashrc"
    else
        warn "Skipped — remember to set ANTHROPIC_API_KEY before running claude"
    fi
fi

# ---------------------------------------------------------------------------
# Done
# ---------------------------------------------------------------------------
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Installation complete."
echo ""
echo "  Open a NEW SSH session (so ~/.bashrc is sourced), then:"
echo ""
echo "    cd ~/auto_navigation_vidalia"
echo "    claude"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
