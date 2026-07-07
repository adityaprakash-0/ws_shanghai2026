#!/bin/bash
# Run on Pi — health check using the sudo ros2 prefix (same as launch).
set -e

WS="${SKILL23_WS:-${HOME}/skill23_ws}"
ROS2_PI="${WS}/src/studica_ws/scripts/ros2_pi.sh"

if [[ ! -x "${ROS2_PI}" ]]; then
  echo "ERROR: ${ROS2_PI} not found — build workspace first"
  exit 1
fi

echo "=== ROS nodes ==="
"${ROS2_PI}" node list 2>/dev/null | grep -E 'control_server|adev_|amcl|controller|planner|ekf' || echo "(no nodes — launch not running?)"

echo ""
echo "=== Battery (/battery_state from VMX power component) ==="
timeout 4 "${ROS2_PI}" topic echo /battery_state --once 2>/dev/null || echo "NOT publishing — is the stack launched with ros2_pi.sh?"

echo ""
echo "=== Odometry (EKF → /odometry/filtered, frame odom) ==="
timeout 4 "${ROS2_PI}" topic echo /odometry/filtered --once 2>/dev/null | head -8 || echo "NOT publishing"

echo ""
echo "=== TF map → base_footprint ==="
timeout 3 "${ROS2_PI}" run tf2_ros tf2_echo map base_footprint 2>/dev/null | head -12 || echo "TF not available"

echo ""
echo "=== Nav2 action ==="
"${ROS2_PI}" action list 2>/dev/null | grep navigate || echo "navigate_to_pose not ready"

echo ""
echo "=== cmd_vel (only active while Nav2 is driving; gamepad disabled for auto nav) ==="
timeout 3 "${ROS2_PI}" topic hz /cmd_vel 2>/dev/null || echo "no traffic (normal when robot is idle)"
