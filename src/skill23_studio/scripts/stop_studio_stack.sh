#!/usr/bin/env bash
# Force-kill Studio nav / camera children (incl. sudo-launched leftovers).
# Does NOT kill studio_bridge / studio.launch (always-on).
# studio_camera_gate no longer exists; its pattern stays below only to
# clean leftovers from installs that predate its removal.
#
# IMPORTANT: never pkill bare package names like "orbbec_camera" — those strings
# appear inside AMENT_PREFIX_PATH on the Studio "sudo -E env …" command line and
# would kill Studio itself.
set -u

echo "studio_stop_stack: reaping nav/camera leftovers…"

# Skip the whole sweep when nothing matches — map/session clicks used to spend
# many seconds here on empty pkill invocations.
if ! pgrep -af -- 'studio_nav|studio_localize|slam_tele_|manual_composition|__node:=control_server|ydlidar_ros2_driver|lidar_merger|lib/skill23_studio/cn_|studio_camera_gate|gemini_e|camera_container|ob_camera_node|bt_navigator|planner_server|controller_server|nav2_container' >/dev/null 2>&1; then
  echo "studio_stop_stack: nothing running"
  exit 0
fi

pkill_pat() {
  local pat="$1"
  # Only invoke pkill when something actually matches this pattern.
  pgrep -af -- "$pat" >/dev/null 2>&1 || return 0
  pkill -9 -f -- "$pat" 2>/dev/null || true
  sudo -n pkill -9 -f -- "$pat" 2>/dev/null || true
}

# Outer launches
pkill_pat 'studio_nav_camera.launch.py'
pkill_pat 'studio_nav2.launch.py'
pkill_pat 'studio_nav.launch.py'
pkill_pat 'studio_localize.launch.py'
pkill_pat 'slam_tele_grip.py'
pkill_pat 'slam_tele_allterrain.py'
pkill_pat 'slam_tele_mecanum.py'
pkill_pat 'slam_tele_xdrive.py'

# Hardware / drive / localization
pkill_pat 'manual_composition'
pkill_pat '__node:=control_server'
pkill_pat 'lib/grip/grip_node'
pkill_pat 'allterrain_node'
pkill_pat 'mecanum_node'
pkill_pat 'xdrive_node'
pkill_pat 'ydlidar_ros2_driver_node'
pkill_pat 'lidar_merger_node'

# Custom-nav nodes
pkill_pat 'lib/skill23_studio/cn_'
pkill_pat 'cn_obstacle_detector'
pkill_pat 'cn_collision_checker'
pkill_pat 'cn_planner'
pkill_pat 'cn_path_smoother'
pkill_pat 'cn_controller'
pkill_pat 'cn_goal_checker'
pkill_pat 'cn_recovery'
pkill_pat 'cn_mission'
pkill_pat 'cn_world_model'
# Stale gate from older installs
pkill_pat 'studio_camera_gate'
pkill_pat '__node:=planner_server'
pkill_pat '__node:=controller_server'
pkill_pat '__node:=bt_navigator'
pkill_pat '__node:=behavior_server'
pkill_pat '__node:=smoother_server'
pkill_pat '__node:=velocity_smoother'
pkill_pat '__node:=nav2_container'
pkill_pat 'scan_filtered_relay'

# Orbbec (specific — do NOT use bare orbbec_camera)
pkill_pat 'gemini_e.launch.py'
pkill_pat 'launch_gemini_e.sh'
pkill_pat '__node:=camera_container'
pkill_pat 'ob_camera_node'

echo "studio_stop_stack: done"
