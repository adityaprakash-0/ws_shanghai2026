#!/bin/bash
# Run this ON YOUR LAPTOP (not on the Pi) to copy the dashboard frontend.
#
# Usage:
#   bash pull_dashboard_from_pi.sh
#   bash pull_dashboard_from_pi.sh 192.168.4.1
#
PI_HOST="${1:-192.168.4.1}"
PI_USER="${2:-vmx}"
DEST="${3:-$HOME/Music/adev_dashboard}"

echo "Pulling ADEV dashboard from ${PI_USER}@${PI_HOST} → ${DEST}"
mkdir -p "$DEST"
scp -r "${PI_USER}@${PI_HOST}:~/skill23_ws/laptop/adev_dashboard/"* "$DEST/"
chmod +x "$DEST/start.sh" 2>/dev/null
echo "Done. On laptop run: cd ${DEST} && ./start.sh"
echo "Then open http://localhost:3000 and set Pi URL to http://${PI_HOST}:8080"
