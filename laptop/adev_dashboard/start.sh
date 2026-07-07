#!/bin/bash
# Run ON YOUR LAPTOP from ~/Music/adev_dashboard
# Serves the UI locally; talks to Pi API over WiFi hotspot.
cd "$(dirname "$0")"
PORT="${1:-3000}"
echo "ADEV Dashboard → http://localhost:${PORT}"
echo "Set Pi Bridge URL in UI to http://<pi-ip>:8080"
python3 -m http.server "$PORT"
