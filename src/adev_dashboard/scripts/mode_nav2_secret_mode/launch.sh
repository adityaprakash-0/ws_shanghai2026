#!/usr/bin/env bash
# nav2_secret_mode — GraphPlanner + RPP
set -eo pipefail

ROBOT="${1:-mecanum}"
MAP="${2:-}"
GRAPH="${3:-}"
WS="${SKILL23_WS:-${HOME}/skill23_ws}"
ROS2_PI="${WS}/src/studica_ws/scripts/ros2_pi.sh"

LAUNCH_ARGS=("robot:=${ROBOT}")
[[ -n "${MAP}" ]] && LAUNCH_ARGS+=("map_file:=${MAP}")
[[ -n "${GRAPH}" ]] && LAUNCH_ARGS+=("graph_file:=${GRAPH}")

exec "${ROS2_PI}" launch adev_dashboard nav2_secret_mode_dashboard.launch.py "${LAUNCH_ARGS[@]}"
