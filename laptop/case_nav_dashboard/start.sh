#!/bin/bash
# Run ON YOUR LAPTOP from laptop/case_nav_dashboard
# Serves the UI locally; talks to Pi case-nav bridge on :8082
cd "$(dirname "$0")"
PORT="${1:-3002}"
echo "Case-Nav Dashboard → http://localhost:${PORT}"
echo "Set Pi Bridge URL in UI to http://<pi-ip>:8082"
echo "(ADEV :8080/:3000, Pick-place :8081/:3001)"
python3 -m http.server "$PORT"
