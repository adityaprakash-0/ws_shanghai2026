#!/usr/bin/env bash
# One-time sudo setup for WorldSkills Studio on the Pi.
# Allows the studio bridge to run HW launches without a password prompt:
#   sudo -E env PATH=… PYTHONPATH=… LD_LIBRARY_PATH=… AMENT_PREFIX_PATH=… CMAKE_PREFIX_PATH=… ros2 …
#
# Also covers studica_control manual_composition (pigpio /dev/mem).
#
# Usage:
#   ./scripts/setup_studio_sudo.sh
#   ./scripts/setup_studio_sudo.sh /path/to/skill23_ws

set -euo pipefail

WS="${1:-$HOME/skill23_ws}"
USER_NAME="${SUDO_USER:-$USER}"
ROS2_BIN="$(command -v ros2 || true)"
if [ -z "$ROS2_BIN" ]; then
  if [ -x /opt/ros/humble/bin/ros2 ]; then
    ROS2_BIN=/opt/ros/humble/bin/ros2
  else
    echo "ERROR: ros2 not found on PATH. Source ROS first."
    exit 1
  fi
fi

EXEC="$WS/install/studica_control/lib/studica_control/manual_composition"
ENV_BIN="$(command -v env)"

echo "Workspace: $WS"
echo "User:      $USER_NAME"
echo "ros2:      $ROS2_BIN"
echo "env:       $ENV_BIN"

if [ -f "$EXEC" ]; then
  STUDICA_SUDOERS=/etc/sudoers.d/studica_control
  echo "$USER_NAME ALL=(ALL) SETENV: NOPASSWD: $EXEC" | sudo tee "$STUDICA_SUDOERS" > /dev/null
  sudo chmod 0440 "$STUDICA_SUDOERS"
  echo "OK  sudoers: manual_composition"
else
  echo "WARN manual_composition not built yet at $EXEC — rebuild studica_control then re-run"
fi

# Passwordless kill/pkill so the bridge + stop_studio_stack.sh can reap
# root-owned nav/camera orphans (launched via sudo) without a password prompt.
# Without this, "Stop nav" / Deploy-stop / Ctrl+C leave manual_composition,
# grip_node, amcl and cn_* running forever.
# Resolve absolute binary paths (sudoers rejects the shell builtin "kill").
if [ -x /usr/bin/kill ]; then KILL_BIN=/usr/bin/kill
elif [ -x /bin/kill ]; then KILL_BIN=/bin/kill
else KILL_BIN="$(command -v kill)"; fi
if [ -x /usr/bin/pkill ]; then PKILL_BIN=/usr/bin/pkill
elif [ -x /bin/pkill ]; then PKILL_BIN=/bin/pkill
else PKILL_BIN="$(command -v pkill)"; fi
KILL_SUDOERS=/etc/sudoers.d/skill23_studio_kill
KTMP="$(mktemp)"
cat > "$KTMP" <<EOF
# WorldSkills Studio — NOPASSWD kill/pkill to reap sudo-launched orphans
$USER_NAME ALL=(ALL) NOPASSWD: $KILL_BIN, $PKILL_BIN
EOF
if sudo visudo -cf "$KTMP" >/dev/null 2>&1; then
  sudo cp "$KTMP" "$KILL_SUDOERS"
  sudo chmod 0440 "$KILL_SUDOERS"
  echo "OK  sudoers: kill/pkill ($KILL_BIN, $PKILL_BIN)"
else
  echo "ERROR: kill/pkill sudoers invalid — check $KILL_BIN / $PKILL_BIN"
fi
rm -f "$KTMP"

# Passwordless: sudo -E env PATH=* PYTHONPATH=* LD_LIBRARY_PATH=* AMENT_PREFIX_PATH=* CMAKE_PREFIX_PATH=* ros2 *
SUDOERS_FILE=/etc/sudoers.d/skill23_studio
TMP="$(mktemp)"
cat > "$TMP" <<EOF
# WorldSkills Studio — NOPASSWD for ros2 via env (SETENV keeps -E working)
Cmnd_Alias STUDIO_ROS2 = $ENV_BIN PATH=* PYTHONPATH=* LD_LIBRARY_PATH=* AMENT_PREFIX_PATH=* CMAKE_PREFIX_PATH=* $ROS2_BIN *
$USER_NAME ALL=(ALL) SETENV: NOPASSWD: STUDIO_ROS2
EOF

if sudo visudo -cf "$TMP" >/dev/null 2>&1; then
  sudo cp "$TMP" "$SUDOERS_FILE"
  sudo chmod 0440 "$SUDOERS_FILE"
  echo "OK  sudoers: $SUDOERS_FILE"
else
  echo "WARN Cmnd_Alias form failed visudo — installing broader fallback"
  echo "$USER_NAME ALL=(ALL) SETENV: NOPASSWD: $ENV_BIN, $ROS2_BIN" | sudo tee "$SUDOERS_FILE" > /dev/null
  sudo chmod 0440 "$SUDOERS_FILE"
  if ! sudo visudo -cf "$SUDOERS_FILE" >/dev/null 2>&1; then
    echo "ERROR: sudoers invalid — remove $SUDOERS_FILE and fix manually"
    rm -f "$TMP"
    exit 1
  fi
  echo "OK  sudoers (fallback): $SUDOERS_FILE"
fi
rm -f "$TMP"

echo
echo "Done. Test (should not ask for a password):"
echo "  sudo -E env PATH=\"\$PATH\" PYTHONPATH=\"\$PYTHONPATH\" LD_LIBRARY_PATH=\"\$LD_LIBRARY_PATH\" AMENT_PREFIX_PATH=\"\$AMENT_PREFIX_PATH\" CMAKE_PREFIX_PATH=\"\$CMAKE_PREFIX_PATH\" ros2 --help"
echo
echo "Then start Studio bridge:"
echo "  source $WS/install/setup.bash"
echo "  ros2 launch skill23_studio studio.launch.py"
