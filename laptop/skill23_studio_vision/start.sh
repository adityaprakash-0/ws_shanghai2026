#!/usr/bin/env bash
# Laptop vision hub. Idle until the Pi sends a start request.
set -euo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$DIR"

set +u
if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

echo
echo "Skill23 Studio Vision  (ROS_DOMAIN_ID=${ROS_DOMAIN_ID})"
echo "  1) aruco   (+ depth approach)"
echo "  2) yolo    (+ depth approach)"
echo "  3) ocr     (+ depth approach)"
echo "  4) barcode (+ depth approach)"
echo "  5) qr      (+ depth approach)"
echo "  6) color   (+ depth approach)"
echo "  7) everything"
echo
read -r -p "Launch which detectors? [1-7] " choice
case "${choice}" in
  1) enable=aruco ;;
  2) enable=yolo ;;
  3) enable=ocr ;;
  4) enable=barcode ;;
  5) enable=qr ;;
  6) enable=color ;;
  7|""|all) enable=aruco,yolo,ocr,barcode,qr,color ;;
  *) echo "Unknown choice"; exit 1 ;;
esac

echo "Starting hub with: ${enable}"
echo "Waiting for Pi /studio/vision/request …"
exec python3 "${DIR}/vision_hub.py" --enable "${enable}"
