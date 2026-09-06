#!/usr/bin/env bash
# Source on a laptop (same WiFi as Pi) to view ROS topics from the robot.
# On the Pi, Studio defaults to ROS_LOCALHOST_ONLY=1. For laptop ↔ Pi ROS,
# start Studio with:  STUDIO_DDS_WIFI=1 ros2 launch skill23_studio studio.launch.py
# Usage:
#   source /opt/ros/humble/setup.bash
#   source /path/to/studio_laptop_ros_env.sh
#   rqt_image_view /studio/detect_align/debug_image

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
export ROS_LOCALHOST_ONLY=0

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS}"
echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
echo "Tip: Pi Studio must use STUDIO_DDS_WIFI=1 for laptop ROS discovery."
