#!/usr/bin/env bash
# Mode 1 — standard Nav2 on Pi (studica_control navigation_*.py)
set -eo pipefail

ROBOT="${1:-allterrain}"
MAP="${2:-}"
WS="${SKILL23_WS:-${HOME}/skill23_ws}"
ROS2_PI="${WS}/src/studica_ws/scripts/ros2_pi.sh"

LAUNCH_ARGS=("robot:=${ROBOT}")
if [[ -n "${MAP}" ]]; then
  LAUNCH_ARGS+=("map_file:=${MAP}")
fi

exec "${ROS2_PI}" launch adev_dashboard nav2_dashboard.launch.py "${LAUNCH_ARGS[@]}"
