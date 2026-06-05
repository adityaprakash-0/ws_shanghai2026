#!/usr/bin/env bash
# nav2 — Theta* + RPP (standard Nav2)
set -eo pipefail

ROBOT="${1:-allterrain}"
MAP="${2:-}"
WS="${SKILL23_WS:-${HOME}/skill23_ws}"
ROS2_PI="${WS}/src/studica_ws/scripts/ros2_pi.sh"

LAUNCH_ARGS=("robot:=${ROBOT}")
[[ -n "${MAP}" ]] && LAUNCH_ARGS+=("map_file:=${MAP}")

exec "${ROS2_PI}" launch adev_dashboard nav2_dashboard.launch.py "${LAUNCH_ARGS[@]}"
