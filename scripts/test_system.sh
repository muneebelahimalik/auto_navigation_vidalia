#!/usr/bin/env bash
# test_system.sh — Step 1: Verify the Amiga brain system environment.
#
# Run this FIRST after SSHing into the brain.
# It checks everything needed before touching Python or the robot.
#
# Usage:
#   bash scripts/test_system.sh
#
# Expected result: all checks print OK.  Any FAIL needs fixing before proceeding.

PASS=0
FAIL=0

ok()   { echo "  [OK]   $1"; ((PASS++)); }
fail() { echo "  [FAIL] $1"; ((FAIL++)); }
info() { echo "  [INFO] $1"; }
sep()  { echo ""; echo "--- $1 ---"; }

echo "========================================"
echo " Amiga brain system check"
echo " $(date)"
echo "========================================"

# -----------------------------------------------------------------------
sep "1. Hostname / identity"
HN=$(hostname)
info "hostname: $HN"
if [[ "$HN" == *"camphor"* ]] || [[ "$HN" == *"amiga"* ]]; then
    ok "Running on expected brain host ($HN)"
else
    info "Hostname '$HN' does not match 'camphor' or 'amiga' — may be a dev PC"
fi

# -----------------------------------------------------------------------
sep "2. Filesystem — overlay / persistence"
if df -h / | grep -q overlay; then
    ok "Overlay filesystem confirmed (/ is overlay — code must live in ~/)"
else
    info "Not an overlay filesystem (may be dev PC)"
fi

HOME_FS=$(df -h "$HOME" | tail -1 | awk '{print $1}')
HOME_FREE=$(df -h "$HOME" | tail -1 | awk '{print $4}')
ok "Home dir ($HOME) is on $HOME_FS — $HOME_FREE free (persistent NVMe)"

if [ -d /farm_ng_image ]; then
    ok "farm-ng image layers present (/farm_ng_image)"
else
    info "/farm_ng_image not found — may be dev PC"
fi

# -----------------------------------------------------------------------
sep "3. .bashrc — check for broken ROS2 source line"
if grep -q "source /opt/ros/foxy/setup.bash" "$HOME/.bashrc" 2>/dev/null; then
    fail ".bashrc still contains 'source /opt/ros/foxy/setup.bash'"
    echo "         Fix:  sed -i '/source \\/opt\\/ros\\/foxy\\/setup.bash/d' ~/.bashrc"
else
    ok ".bashrc does NOT source /opt/ros/foxy/setup.bash (correct)"
fi

if grep -q "npm-global" "$HOME/.bashrc" 2>/dev/null; then
    info ".bashrc still has npm-global PATH entry (can be removed; NVM handles Node)"
fi

NVM_IN_BASHRC=$(grep -c "NVM_DIR" "$HOME/.bashrc" 2>/dev/null || echo 0)
if [[ "$NVM_IN_BASHRC" -ge 2 ]]; then
    ok ".bashrc has NVM lines ($NVM_IN_BASHRC occurrences)"
else
    fail ".bashrc is missing NVM setup lines (Node.js may not persist after reboot)"
    echo "         Add to ~/.bashrc:"
    echo '         export NVM_DIR="$HOME/.nvm"'
    echo '         [ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"'
fi

# -----------------------------------------------------------------------
sep "4. NVM / Node.js"
if [ -d "$HOME/.nvm" ]; then
    ok "NVM installed at ~/.nvm (persistent)"
else
    fail "NVM not found at ~/.nvm"
fi

if command -v node &>/dev/null; then
    NODE_VER=$(node --version)
    ok "Node.js available: $NODE_VER"
else
    fail "node not on PATH — source ~/.bashrc or run: export NVM_DIR=\"\$HOME/.nvm\" && source \"\$NVM_DIR/nvm.sh\""
fi

if command -v npm &>/dev/null; then
    ok "npm available: $(npm --version)"
else
    fail "npm not on PATH"
fi

# -----------------------------------------------------------------------
sep "5. Claude Code binary"
CLAUDE_BIN="$HOME/.npm-global/bin/claude"
if [ -f "$CLAUDE_BIN" ]; then
    ok "Claude Code binary found at $CLAUDE_BIN"
elif command -v claude &>/dev/null; then
    ok "Claude Code on PATH: $(command -v claude)"
else
    fail "Claude Code binary not found"
fi

# -----------------------------------------------------------------------
sep "6. Python 3"
if command -v python3 &>/dev/null; then
    PY_VER=$(python3 --version)
    ok "python3 available: $PY_VER"
else
    fail "python3 not on PATH"
fi

# -----------------------------------------------------------------------
sep "7. farm-ng venv"
FARMNG_VENV="/farm_ng_image/venv"
if [ -f "$FARMNG_VENV/bin/activate" ]; then
    ok "farm-ng venv found: $FARMNG_VENV"
    FARMNG_PY="$FARMNG_VENV/bin/python3"
    FARMNG_VER=$("$FARMNG_PY" --version 2>&1)
    ok "farm-ng Python: $FARMNG_VER"
else
    fail "farm-ng venv not found at $FARMNG_VENV"
    echo "         Install farm-ng SDK with:  bash scripts/install_farmng.sh"
fi

# -----------------------------------------------------------------------
sep "8. ROS 2 (expected: NOT present on brain)"
if [ -f /opt/ros/foxy/setup.bash ]; then
    info "ROS 2 Foxy IS present — this may be a dev PC"
    info "On the brain this would be wiped on reboot (overlay FS)"
else
    ok "ROS 2 Foxy NOT installed (correct for Amiga brain — uses native Python stack)"
fi

# -----------------------------------------------------------------------
sep "9. Tailscale connectivity"
if command -v tailscale &>/dev/null; then
    TS_STATUS=$(tailscale status --json 2>/dev/null | python3 -c "import sys,json; d=json.load(sys.stdin); print('connected' if d.get('BackendState')=='Running' else d.get('BackendState','unknown'))" 2>/dev/null || echo "unknown")
    if [[ "$TS_STATUS" == "connected" ]]; then
        ok "Tailscale connected"
    else
        fail "Tailscale status: $TS_STATUS"
        echo "         Start with:  sudo tailscale up"
    fi
else
    info "tailscale CLI not found — check via:  systemctl status tailscaled"
fi

# -----------------------------------------------------------------------
sep "10. Workspace directory"
WS="$HOME/auto_navigation_vidalia"
if [ -d "$WS" ]; then
    ok "Workspace found: $WS"
else
    fail "Workspace NOT found at $WS"
    echo "         Clone with:  git clone <repo-url> ~/auto_navigation_vidalia"
fi

if [ -f "$WS/main.py" ]; then
    ok "main.py present"
else
    fail "main.py missing — git pull in $WS"
fi

if [ -f "$WS/service_config.json" ]; then
    ok "service_config.json present"
else
    fail "service_config.json missing"
fi

if [ -f "$WS/requirements.txt" ]; then
    ok "requirements.txt present"
else
    fail "requirements.txt missing"
fi

# -----------------------------------------------------------------------
echo ""
echo "========================================"
echo " Results: $PASS passed, $FAIL failed"
echo "========================================"
if [[ "$FAIL" -eq 0 ]]; then
    echo " All checks passed."
    echo " Next step:  python3 scripts/test_sdk.py"
else
    echo " Fix the FAIL items above, then re-run this script."
fi
echo ""
