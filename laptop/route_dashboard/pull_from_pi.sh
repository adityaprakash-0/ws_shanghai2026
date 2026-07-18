#!/bin/bash
# Pull route_dashboard folder from Pi to laptop (same layout as adev_dashboard)
PI_IP="${1:-192.168.49.1}"
DEST="${2:-$HOME/Music/laptop/route_dashboard}"
mkdir -p "$(dirname "$DEST")"
scp -r "vmx@${PI_IP}:~/skill23_ws/laptop/route_dashboard/"* "$DEST/"
chmod +x "$DEST/start.sh"
echo "Done → $DEST"
echo "Run: cd $DEST && ./start.sh"
