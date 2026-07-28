# skill23_case_nav

Case-based lightweight navigation **without Nav2**. Pure **Python (rclpy)** — no C++ build.

Separate from ADEV (`:8080`) and pick-place (`:8081`). Bridge: **`:8082`**.

## Sensors

| Role | Sensors | Topics |
|------|---------|--------|
| Front obstacle | Front lidar FOV ±45° | `/scan/front` |
| Walls / open | Left & right ultrasonic | `/ultrasonic_left/range`, `/ultrasonic_right/range` |
| Recovery map update | Dual lidar merger | `/scan` |

Sharp IR is disabled in HW params. Localization uses EKF (`/wheel/odom` + `/imu` → `/odometry/filtered`).

## Launch (on Pi)

```bash
cd ~/skill23_ws
source /opt/ros/humble/setup.bash
# Light install (Python only — safe on Pi, no C++ compile)
colcon build --packages-select skill23_case_nav --symlink-install
source install/setup.bash

ros2 launch skill23_case_nav case_nav.launch.py \
  robot:=mecanum \
  map:=~/skill23_ws/test_map11.yaml
```

Do **not** run together with ADEV / Nav2 / `slam_tele_mecanum` (same HW + `/cmd_vel`).

## Dashboard (laptop)

```bash
cd ~/skill23_ws/laptop/case_nav_dashboard
./start.sh          # http://localhost:3002
```

Set bridge URL to `http://<pi-ip>:8082` → Connect → Load map → Set Pose → Goal(s) → Start.

## Config files

| File | Purpose |
|------|---------|
| `config/hw_params_mecanum.yaml` (also `_xdrive`, `_allterrain`) | Studica HW: Titan, US, IMU, light tower |
| `config/case_nav_mecanum.yaml` (also `_xdrive`, `_allterrain`) | Speeds, FOV, planner, recovery |
| `config/ekf_case_nav.yaml` | EKF for `/odometry/filtered` |
