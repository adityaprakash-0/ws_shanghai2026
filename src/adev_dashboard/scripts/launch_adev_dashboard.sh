#!/usr/bin/env bash
# Launch ADEV dashboard stack using the Pi sudo prefix.
#
# Usage:
#   ./scripts/launch_adev_dashboard.sh [robot] [map_yaml]
#   ./scripts/launch_adev_dashboard.sh --laptop-nav2 [robot] [map_yaml]
#
# --laptop-nav2  Pi runs hardware/sensors only; start Nav2 on laptop separately
#                (see ~/skill23_ws/laptop/start_laptop_nav2.sh).
#
# Examples:
#   ./scripts/launch_adev_dashboard.sh allterrain
#   ./scripts/launch_adev_dashboard.sh --laptop-nav2 mecanum

set -eo pipefail

NAV2_ON_LAPTOP=false
if [[ "${1:-}" == "--laptop-nav2" ]]; then
  NAV2_ON_LAPTOP=true
  shift
fi

ROBOT="${1:-allterrain}"
MAP="${2:-}"
WS="${SKILL23_WS:-${HOME}/skill23_ws}"
ROS2_PI="${WS}/src/studica_ws/scripts/ros2_pi.sh"

if [[ ! -x "${ROS2_PI}" ]]; then
  echo "ERROR: ${ROS2_PI} not found or not executable"
  exit 1
fi

LAUNCH_ARGS=("robot:=${ROBOT}" "nav2_on_laptop:=${NAV2_ON_LAPTOP}")
if [[ -n "${MAP}" ]]; then
  LAUNCH_ARGS+=("map_file:=${MAP}")
fi

exec "${ROS2_PI}" launch adev_dashboard adev_dashboard.launch.py "${LAUNCH_ARGS[@]}"
