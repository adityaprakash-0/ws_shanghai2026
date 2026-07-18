#!/usr/bin/env bash
# Mode 2 — Nav2 + wheel IR/ultrasonic safety guard (skill23_nav_guard)
set -eo pipefail

ROBOT="${1:-mecanum}"
MAP="${2:-}"
WS="${SKILL23_WS:-${HOME}/skill23_ws}"
ROS2_PI="${WS}/src/studica_ws/scripts/ros2_pi.sh"

if [[ "${ROBOT}" != "mecanum" && "${ROBOT}" != "xdrive" ]]; then
  echo "ERROR: nav2_guard mode only supports mecanum or xdrive"
  exit 1
fi

LAUNCH_ARGS=("robot:=${ROBOT}")
if [[ -n "${MAP}" ]]; then
  LAUNCH_ARGS+=("map_file:=${MAP}")
fi

exec "${ROS2_PI}" launch adev_dashboard nav2_guard_dashboard.launch.py "${LAUNCH_ARGS[@]}"
