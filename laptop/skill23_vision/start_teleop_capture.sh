#!/usr/bin/env bash
# Laptop: joy_node + teleop YOLO/AprilTag capture
# Same ROS_DOMAIN_ID as the Pi. Gamepad plugged into THIS laptop.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi

YOLO_BTN="${YOLO_BTN:-0}"   # A  — change if your pad differs
TAG_BTN="${TAG_BTN:-1}"     # B  — change if your pad differs
OUT="${OUT:-$ROOT/captures}"

echo "[skill23_vision] domain=$ROS_DOMAIN_ID  YOLO_BTN=$YOLO_BTN  TAG_BTN=$TAG_BTN"
echo "[skill23_vision] Tip: find button indices with:  ros2 topic echo /joy"
echo "[skill23_vision] starting joy_node + teleop_capture (Ctrl+C stops both)"

ros2 run joy joy_node &
JOY_PID=$!
trap 'kill $JOY_PID 2>/dev/null || true' EXIT

sleep 1
python3 "$ROOT/teleop_capture.py" --out "$OUT" --yolo-btn "$YOLO_BTN" --tag-btn "$TAG_BTN"
