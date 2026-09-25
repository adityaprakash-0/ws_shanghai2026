#!/usr/bin/env bash
# Keyboard backup teleop — NO joy_node (avoids fighting /cmd_vel with gamepad).
# Pi: teleop_*.launch.py already running. Laptop: this script only.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
cd "$ROOT"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi

echo "[keyboard_teleop] domain=$ROS_DOMAIN_ID  (do NOT run joy_node at the same time)"
python3 "$ROOT/keyboard_teleop.py" "$@"
