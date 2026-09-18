# Skill23 EKF calibration — how to run tests per robot

Copy-paste commands for open-loop odometry, IMU, and `ekf_covariance` YAML generation.
Reports are written under `src/skill23_test/reports/` (after install: `share/skill23_test/reports` via `report_paths`).

Robots: `grip` | `mecanum` | `xdrive` | `allterrain`

Requires passwordless/`sudo` for Titan `manual_composition` (same as Studio HW).

---

## 0. Workspace setup (once)

```bash
cd ~/skill23_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select skill23_test --symlink-install
source install/setup.bash
```

---

## 1. Wheel odometry calibration (per robot)

**Terminal A — hardware + drive (+ optional EKF):**

```bash
ros2 launch skill23_test odom_test.launch.py robot:=xdrive
# robot:=grip | mecanum | xdrive | allterrain
```

**Terminal B — interactive calibrator:**

```bash
ros2 run skill23_test odom_calibrator --ros-args -p robot:=xdrive
```

Follow the prompts (drive forward / strafe / turn). JSON + Markdown reports appear as:

`reports/<robot>_odom_cal_<timestamp>.json`  
`reports/<robot>_odom_cal_<timestamp>.md`

Repeat with `robot:=grip`, `mecanum`, `allterrain` as needed.

---

## 2. IMU calibration (raw `/imu`, no EKF)

Usually one IMU report is shared across chassis (same board).

**Terminal A:**

```bash
ros2 launch skill23_test imu_test.launch.py robot:=xdrive
```

**Terminal B:**

```bash
ros2 run skill23_test imu_calibrator --ros-args -p robot:=xdrive
```

Output example:

`reports/xdrive_imu_cal_<timestamp>.json`  
`reports/xdrive_imu_cal_<timestamp>.md`

---

## 3. Build EKF P / Q diagonals (`ekf_covariance`)

Edit the report paths at the top of:

`src/skill23_test/skill23_test/ekf_covariance.py`

```python
ODOM_REPORT = REPORTS / 'allterrain_odom_cal_20260825_140730.json'
IMU_REPORT  = REPORTS / 'xdrive_imu_cal_20260823_185134.json'
```

Uncomment the matching odom file for the robot you want:

| Robot        | Typical ODOM_REPORT                              | IMU_REPORT                         |
|--------------|--------------------------------------------------|------------------------------------|
| allterrain   | `allterrain_odom_cal_*.json`                     | `xdrive_imu_cal_*.json` (shared)   |
| grip         | `grip_odom_cal_*.json` (or `*_corrected.json`)   | same IMU                           |
| mecanum      | `mecanum_odom_cal_*.json`                        | same IMU                           |
| xdrive       | `xdrive_odom_cal_*.json`                         | `xdrive_imu_cal_*.json`            |

Then run:

```bash
ros2 run skill23_test ekf_covariance
# or: python3 -m skill23_test.ekf_covariance
```

Outputs:

- `reports/ekf_<robot>/ekf_covariance.md` — copy-paste YAML blocks  
- `reports/ekf_<robot>/ekf_covariance.json` — machine-readable

Paste the diagonals into the robot EKF config, e.g.:

- `src/allterrain/config/ekf_allterrain.yaml`
- `src/grip/config/ekf_grip.yaml`
- `src/mecanum/config/ekf_mecanum.yaml`
- `src/xdrive/config/ekf_xdrive.yaml`

Holonomic (`mecanum`, `xdrive`): report includes full + `_diff` (vy unused) blocks.  
Skid-steer (`grip`, `allterrain`): single block (no vy).

---

## Quick cheat sheet

```bash
# Odom — Terminal A / B
ros2 launch skill23_test odom_test.launch.py robot:=ROBOT
ros2 run skill23_test odom_calibrator --ros-args -p robot:=ROBOT

# IMU — Terminal A / B
ros2 launch skill23_test imu_test.launch.py robot:=ROBOT
ros2 run skill23_test imu_calibrator --ros-args -p robot:=ROBOT

# EKF YAML from reports (edit paths in ekf_covariance.py first)
ros2 run skill23_test ekf_covariance
```

Replace `ROBOT` with `grip`, `mecanum`, `xdrive`, or `allterrain`.
