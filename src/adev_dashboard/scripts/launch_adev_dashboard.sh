#!/usr/bin/env bash
# ADEV dashboard launcher — pick navigation mode interactively.
#
# Modes:
#   nav2              — Full Nav2 on Pi (allterrain / mecanum / xdrive)
#   nav2_guard        — Nav2 + wheel sensor safety guard (mecanum / xdrive only)
#   nav2_secret_mode  — GeoJSON GraphPlanner + stock BT + RPP
#
# Usage:
#   ./scripts/launch_adev_dashboard.sh
#   ./scripts/launch_adev_dashboard.sh nav2 mecanum
#   ./scripts/launch_adev_dashboard.sh nav2_secret_mode mecanum ~/skill23_ws/test_map11.yaml

set -eo pipefail

WS="${SKILL23_WS:-${HOME}/skill23_ws}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

MODE="${1:-}"
ROBOT="${2:-}"
MAP="${3:-}"
GRAPH="${4:-}"

if [[ -z "${MODE}" ]]; then
  echo ""
  echo "=== ADEV Dashboard — select mode ==="
  echo "  1) nav2              — standard Nav2"
  echo "  2) nav2_guard        — Nav2 + wheel IR/US safety guard"
  echo "  3) nav2_secret_mode  — GraphPlanner corridor navigation"
  echo ""
  read -r -p "Mode [1/2/3]: " choice
  case "${choice}" in
    1|nav2) MODE="nav2" ;;
    2|nav2_guard) MODE="nav2_guard" ;;
    3|nav2_secret_mode|secret_mode) MODE="nav2_secret_mode" ;;
    *) echo "Invalid choice"; exit 1 ;;
  esac
fi

# Accept short alias
if [[ "${MODE}" == "secret_mode" ]]; then
  MODE="nav2_secret_mode"
fi

if [[ -z "${ROBOT}" ]]; then
  if [[ "${MODE}" == "nav2_guard" ]]; then
    read -r -p "Robot [mecanum/xdrive] (default mecanum): " ROBOT
    ROBOT="${ROBOT:-mecanum}"
  else
    read -r -p "Robot [allterrain/mecanum/xdrive] (default allterrain): " ROBOT
    ROBOT="${ROBOT:-allterrain}"
  fi
fi

GRAPH="${GRAPH:-}"
if [[ "${MODE}" == "nav2_secret_mode" ]]; then
  if [[ -z "${MAP}" && -z "${4:-}" ]]; then
    read -r -p "Map yaml path (optional, Enter for default): " MAP
    read -r -p "Graph geojson path (optional, Enter for auto from map name): " GRAPH
  fi
elif [[ -z "${MAP}" ]]; then
  read -r -p "Map yaml path (optional, Enter for default): " MAP
fi

case "${MODE}" in
  nav2)
    exec "${SCRIPT_DIR}/mode_nav2/launch.sh" "${ROBOT}" "${MAP}"
    ;;
  nav2_guard)
    exec "${SCRIPT_DIR}/mode_nav2_guard/launch.sh" "${ROBOT}" "${MAP}"
    ;;
  nav2_secret_mode)
    exec "${SCRIPT_DIR}/mode_nav2_secret_mode/launch.sh" "${ROBOT}" "${MAP}" "${GRAPH}"
    ;;
  *)
    echo "Unknown mode: ${MODE} (use nav2, nav2_guard, or nav2_secret_mode)"
    exit 1
    ;;
esac
