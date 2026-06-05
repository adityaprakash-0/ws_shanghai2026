#!/bin/bash
# Run ON YOUR LAPTOP from laptop/pick_place_dashboard
# Serves the UI locally; talks to Pi PP bridge on :8081
cd "$(dirname "$0")"
PORT="${1:-3001}"
echo "Pick & Place Dashboard → http://localhost:${PORT}"
echo "Set Pi Bridge URL in UI to http://<pi-ip>:8081"
echo "(ADEV Nav2 dashboard stays on :8080 / laptop :3000)"
python3 -m http.server "$PORT"
