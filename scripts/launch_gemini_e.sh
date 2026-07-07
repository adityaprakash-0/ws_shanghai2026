#!/usr/bin/env bash
# Launch Orbbec Gemini E from workspace local_sysroot (not global apt).
# Runs as the current user (no sudo) — avoids /dev/shm orbbec_device_lock issues.
set -eo pipefail

WS="${SKILL23_WS:-${HOME}/skill23_ws}"

set +u
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"
if [ -f "${WS}/local_sysroot/env.bash" ]; then
  # shellcheck source=/dev/null
  source "${WS}/local_sysroot/env.bash"
fi
set -u

# Remove stale lock from a crashed or sudo-launched prior run.
rm -f /dev/shm/orbbec_device_lock

exec ros2 launch orbbec_camera gemini_e.launch.py "$@"
