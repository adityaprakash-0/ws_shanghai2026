#!/usr/bin/env bash
# ADEV dashboard — two navigation modes only.
#
#   nav2              — Theta* planner + RPP (dynamic free-space)
#   nav2_secret_mode  — GraphPlanner (GeoJSON) + RPP (fixed corridors)
#
# Usage:
#   ./scripts/launch_adev_dashboard.sh
#   ./scripts/launch_adev_dashboard.sh nav2 mecanum
#   ./scripts/launch_adev_dashboard.sh nav2_secret_mode mecanum ~/skill23_ws/test_map11.yaml

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

MODE="${1:-}"
ROBOT="${2:-}"
MAP="${3:-}"
GRAPH="${4:-}"

if [[ -z "${MODE}" ]]; then
  echo ""
  echo "=== ADEV Dashboard — select mode ==="
  echo "  1) nav2              — Theta* + RPP (dynamic)"
  echo "  2) nav2_secret_mode  — GraphPlanner + RPP (corridors)"
  echo ""
  read -r -p "Mode [1/2]: " choice
  case "${choice}" in
    1|nav2) MODE="nav2" ;;
    2|nav2_secret_mode|secret_mode) MODE="nav2_secret_mode" ;;
    *) echo "Invalid choice"; exit 1 ;;
  esac
fi

# Short alias
if [[ "${MODE}" == "secret_mode" ]]; then
  MODE="nav2_secret_mode"
fi

if [[ -z "${ROBOT}" ]]; then
  read -r -p "Robot [allterrain/mecanum/xdrive/grip] (default allterrain): " ROBOT
  ROBOT="${ROBOT:-allterrain}"
fi

if [[ "${MODE}" == "nav2_secret_mode" ]]; then
  if [[ -z "${MAP}" ]]; then
    read -r -p "Map yaml path (optional, Enter for default): " MAP
    read -r -p "Graph geojson path (optional, Enter to auto-detect): " GRAPH
  fi
elif [[ -z "${MAP}" ]]; then
  read -r -p "Map yaml path (optional, Enter for default): " MAP
fi

case "${MODE}" in
  nav2)
    exec "${SCRIPT_DIR}/mode_nav2/launch.sh" "${ROBOT}" "${MAP}"
    ;;
  nav2_secret_mode)
    exec "${SCRIPT_DIR}/mode_nav2_secret_mode/launch.sh" "${ROBOT}" "${MAP}" "${GRAPH}"
    ;;
  *)
    echo "Unknown mode: ${MODE} (use nav2 or nav2_secret_mode)"
    exit 1
    ;;
esac
