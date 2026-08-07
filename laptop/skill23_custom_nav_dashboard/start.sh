#!/bin/bash
# Run ON YOUR LAPTOP from laptop/skill23_custom_nav_dashboard
# Serves the UI locally; talks to Pi custom-nav bridge on :8082
#
# Aligned with skill23_custom_nav:
#   Start → mission → plan_request → path → controller (lookahead)
#   Collision (footprint vs local −99/100) → recovery → local EDT → replan
cd "$(dirname "$0")"
PORT="${1:-3002}"
echo "Custom-Nav Dashboard → http://localhost:${PORT}"
echo "Set Pi Bridge URL in UI to http://<pi-ip>:8082"
echo "(ADEV :8080/:3000, Pick-place :8081/:3001, Custom-Nav :8082/:3002)"
python3 -m http.server "$PORT"
