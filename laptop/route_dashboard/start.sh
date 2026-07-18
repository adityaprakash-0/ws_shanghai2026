#!/bin/bash
# Run ON YOUR LAPTOP from ~/Music/laptop/route_dashboard
cd "$(dirname "$0")"
PORT="${1:-3001}"
echo "Route Graph Builder → http://localhost:${PORT}"
echo "Load yaml+pgm · build graph · Export GeoJSON · scp to Pi"
echo "Missions: use ADEV dashboard (port 3000) with Pi nav2_route mode"
python3 -m http.server "$PORT"
