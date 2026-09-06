#!/usr/bin/env bash
# Serve Skill23 Studio UI on :3003
cd "$(dirname "$0")"
PORT="${1:-3003}"
echo "Connect UI to Pi bridge at http://10.42.0.1:8083"
exec python3 serve.py "$PORT"
