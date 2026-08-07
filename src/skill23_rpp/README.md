# skill23_rpp

Vendored **Navigation2 Humble Regulated Pure Pursuit**, packaged as `skill23_rpp`
for the custom navigation stack (same pattern as `skill23_amcl`).

- ROS package / executable: `skill23_rpp` / `rpp`
- Plain `rclcpp::Node` — `main` → `start()` → `spin` (no lifecycle manager)
- Stock `use_rotate_to_heading` (single ω — no dual fast/slow)
- Costmap from `/custom_nav/local_grid` (`Costmap2D`, not `Costmap2DROS`)
- Collision stop via `/custom_nav/collision`; RPP collision detection off by default

## Build

```bash
cd ~/skill23_ws
colcon build --packages-select skill23_rpp --symlink-install
source install/setup.bash
```

## I/O

| Topic | Role |
|-------|------|
| `/custom_nav/controller/enable` | Gate `/cmd_vel` |
| `/custom_nav/path` | Plan |
| `/amcl_pose` + TF | Pose |
| `/odometry/filtered` | Velocity |
| `/custom_nav/local_grid` | Local Costmap2D |
| `/custom_nav/collision` | Stop when blocked |
| `/custom_nav/goal_status` | Idle on `GOAL_REACHED` |
| `/cmd_vel` | Commands |

Upstream license: Apache-2.0 (Nav2 RPP).
