#!/usr/bin/env bash
# Run ros2 on the Pi with sudo -E and preserved library paths.
# VMX hardware (manual_composition) runs as root; launching the whole stack
# this way keeps DDS discovery and topic access consistent (battery, IMU, etc.).
#
# Usage:
#   ./scripts/ros2_pi.sh launch studica_control navigation_allterrain.py
#   ./scripts/ros2_pi.sh launch adev_dashboard adev_dashboard.launch.py robot:=allterrain
#   ./scripts/ros2_pi.sh topic list
#   ./scripts/ros2_pi.sh topic echo /battery_state --once
#
# Optional: add to ~/.bashrc after sourcing the workspace:
#   alias ros2_pi='~/skill23_ws/src/studica_ws/scripts/ros2_pi.sh'

set -eo pipefail

WS="${SKILL23_WS:-${HOME}/skill23_ws}"

# ROS setup.bash references vars that may be unset; avoid nounset during source.
set +u
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"
if [ -f "${WS}/local_sysroot/env.bash" ]; then
  # shellcheck source=/dev/null
  source "${WS}/local_sysroot/env.bash"
fi
set -u

exec sudo -E env \
  PATH="${PATH}" \
  PYTHONPATH="${PYTHONPATH:-}" \
  LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}" \
  AMENT_PREFIX_PATH="${AMENT_PREFIX_PATH:-}" \
  CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH:-}" \
  ros2 "$@"
