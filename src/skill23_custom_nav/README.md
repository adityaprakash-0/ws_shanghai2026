# skill23_custom_nav

Lightweight custom navigation stack for ROS 2 Humble — **no Nav2 stack**. Localization via
vendored C++ **`skill23_amcl`** (Navigation2 Humble `nav2_amcl` fork).

**Build policy:** edit all code first, then build packages **one at a time** at the end
(no mid-edit rebuilds).

## Quick start

```bash
cd ~/skill23_ws
# Sequential builds (do once after changes)
colcon build --packages-select skill23_amcl --symlink-install
colcon build --packages-select skill23_custom_nav --symlink-install
# Optional YAML install into robot shares (or symlink):
colcon build --packages-select allterrain --symlink-install
colcon build --packages-select mecanum --symlink-install
colcon build --packages-select xdrive --symlink-install
colcon build --packages-select grip --symlink-install

source install/setup.bash
ros2 launch skill23_custom_nav custom_nav.launch.py \
  robot:=grip \
  map:=~/skill23_ws/test_map11.yaml
```

Dashboard (laptop): `laptop/skill23_custom_nav_dashboard/start.sh` → http://localhost:3002  
HTTP API on Pi **:8082**.

## Architecture

| Node | Role |
|------|------|
| `cn_world_model` | Load/publish `/map`, compute static EDT, `/custom_nav/world/ready` |
| `cn_obstacle_detector` | Lidar → rolling `/custom_nav/local_grid` |
| `cn_collision_checker` | Forward zone → `/custom_nav/collision` JSON |
| `cn_planner` | EDT-preferring A* + straight string-pull → `/custom_nav/path` |
| `cn_controller` | Closed-loop AMCL path executor (fixed duties) → `/cmd_vel` |
| `cn_goal_checker` | `/custom_nav/goal_status` |
| `cn_recovery` | Blocked → wait/replan/fail |
| `cn_mission` | Waypoints, start/stop, controller enable |
| `cn_bridge` | HTTP :8082 dashboard bridge |

## Topics

**Standard:** `/map`, `/scan`, `/odometry/filtered`, `/amcl_pose`, `/initialpose`, `/cmd_vel`

**Custom nav:**
- `/custom_nav/local_grid`, `/custom_nav/path`, `/custom_nav/goal`, `/custom_nav/goals`
- `/custom_nav/collision`, `/custom_nav/goal_status`, `/custom_nav/recovery`
- `/custom_nav/plan_request`, `/custom_nav/replan`, `/custom_nav/current_goal`
- `/custom_nav/controller/enable`, `/custom_nav/static_edt`, `/custom_nav/world/ready`
- `/custom_nav/mission/start`, `/custom_nav/mission/stop`, `/custom_nav/mission/status`

## Parameters

Software params: `<robot_pkg>/config/custom_nav2_param.yaml` only
(e.g. `grip/config/custom_nav2_param.yaml`).

Hardware params: `skill23_custom_nav/config/hw_params_<robot>.yaml`.

EKF: `skill23_custom_nav/config/ekf_custom_nav.yaml`.

## Path shape

Like the black path in sim (not orange corner-cuts):

1. A* strongly prefers cells **farthest from walls** (static EDT).
2. Snap each cell to the local EDT maximum (corridor / room center).
3. Straight string-pull **only if** the chord keeps ~90% of the ridge path's
   clearance — so shortcuts that hug inner corners are rejected, and the path
   swings **out into open space** instead.

## Controller (`cn_controller`)

Closed-loop AMCL path executor (lives in this package only — no separate follower/RPP pkg):

- States: idle → align → follow → final_yaw → done (or blocked on collision).
- Fixed `linear_vel` / `angular_vel` / `fine_angular_vel` (on grip these are Titan duties −1…1).
- Large heading error → stop and rotate in place; small errors steer while driving.
- Collision → blocked until a **new path** arrives (recovery owns replan).
- Goal XY/yaw tolerances live only in **`cn_goal_checker`**.
- `POSITION_REACHED` → final yaw rotate; `GOAL_REACHED` → stop and wait for mission.

Tune in each robot’s `config/custom_nav2_param.yaml` under `cn_controller:`.

AMCL on the dashboard: when the robot is still, AMCL may stop republishing.
`pose held · no new msg …s` means the last pose is still used (not “lost”).

## Mission flow

1. Publish goal: `PoseStamped` on `/custom_nav/goal` or JSON waypoints on `/custom_nav/goals`.
2. `ros2 topic pub --once /custom_nav/mission/start std_msgs/msg/Empty {}`
3. Mission manager sends `plan_request`, enables controller, monitors `goal_status`.
4. On collision: recovery waits, publishes `replan`, repeats up to `max_recovery_attempts`.

## Tests

```bash
cd ~/skill23_ws/src/skill23_custom_nav
python3 -m pytest test/test_edt_astar.py -v
```

## Dependencies

- `skill23_amcl` (executable `amcl`)
- `studica_control`, robot drive package, `robot_localization`, `ydlidar_ros2_driver`, `allterrain` (lidar merger)
