#!/usr/bin/env bash
# Run ON YOUR LAPTOP — Nav2 only (MPPI / tuned params in laptop_nav2_ws).
# Pi must already run the matching *pi bringup* launch over hotspot.
#
# Usage:
#   ./start_laptop_nav2.sh allterrain [map.yaml]
#   ./start_laptop_nav2.sh mecanum
#
# Prerequisites (both machines):
#   export ROS_DOMAIN_ID=0          # same value on Pi and laptop
#   export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
#   Laptop WiFi connected to Pi hotspot
#
# Pi side (example):
#   ~/skill23_ws/src/adev_dashboard/scripts/launch_adev_dashboard.sh --laptop-nav2 allterrain

set -eo pipefail

ROBOT="${1:-allterrain}"
MAP="${2:-}"

WS="${LAPTOP_NAV2_WS:-${HOME}/laptop_nav2_ws}"

if [[ ! -f "${WS}/install/setup.bash" ]]; then
  echo "ERROR: laptop_nav2_ws not built at ${WS}"
  echo "  cd ${WS} && colcon build --packages-select laptop_nav2"
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_LOCALHOST_ONLY=0

case "${ROBOT}" in
  allterrain)
    LAUNCH_PKG=laptop_nav2
    LAUNCH_FILE=laptop_nav2_allterrain.py
    DEFAULT_MAP="${HOME}/Music/test_map10.yaml"
    ;;
  mecanum)
    LAUNCH_PKG=laptop_nav2
    LAUNCH_FILE=laptop_nav2_mecanum.py
    DEFAULT_MAP="${HOME}/Music/test_map9.yaml"
    ;;
  *)
    echo "Unknown robot '${ROBOT}'. Use: allterrain | mecanum"
    exit 1
    ;;
esac

MAP="${MAP:-${DEFAULT_MAP}}"

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "Launching Nav2 on laptop: ${LAUNCH_PKG}/${LAUNCH_FILE}"
echo "Map: ${MAP}"
echo ""
echo "Verify Pi topics from laptop:"
echo "  ros2 topic hz /scan"
echo "  ros2 topic hz /odometry/filtered"
echo "  ros2 action list | grep navigate"

exec ros2 launch "${LAUNCH_PKG}" "${LAUNCH_FILE}" "map:=${MAP}"
