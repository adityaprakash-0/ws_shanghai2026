# Competition patch — teleop pose + YOLO/AprilTag button capture

Offline copy kit for a competition workspace **without internet**.

Copy these files into your Pi `skill23_ws` and laptop `skill23_vision` folders
as listed below. **Does not modify** the Docker Pi package
`src/skill23_vision` (vision_gate) — laptop scripts are a separate folder.

---

## What you get

### On the Pi

1. **simple_pose** node — starts at `(0, 0, 0)`, tracks **x, y, yaw** from
   encoder + IMU odometry (`/wheel/odom`), publishes `/robot_pose`
2. **Teleop launch** (all 4 robots) — HW with **gamepad enabled** + drive node
   + simple_pose + Orbbec camera (color streaming)
3. **Strafe** gamepad axes enabled for **mecanum** and **xdrive**
   (grip / allterrain stay diff-drive, no strafe)

### On the laptop

1. Gamepad → `joy_node` → `/joy` (Pi gamepad component drives the robot)
2. **Button A** → grab camera frame → **YOLO** → show boxes/class/pose/time → save
3. **Button B** → grab camera frame → **AprilTag** → show id/pose/time → save
4. Detection runs **only** when you press the button

---

## Files to copy / replace

Paths below are relative to this patch folder → destination on your machines.

### Pi — `~/skill23_ws` (or your competition studio workspace)

| This patch file | Copy to |
|---|---|
| `skill23_studio/skill23_studio/simple_pose_node.py` | `src/skill23_studio/skill23_studio/simple_pose_node.py` **(NEW)** |
| `skill23_studio/launch/teleop_arena.launch.py` | `src/skill23_studio/launch/teleop_arena.launch.py` **(NEW)** |
| `skill23_studio/launch/teleop_mecanum.launch.py` | `src/skill23_studio/launch/teleop_mecanum.launch.py` **(NEW)** |
| `skill23_studio/launch/teleop_xdrive.launch.py` | `src/skill23_studio/launch/teleop_xdrive.launch.py` **(NEW)** |
| `skill23_studio/launch/teleop_grip.launch.py` | `src/skill23_studio/launch/teleop_grip.launch.py` **(NEW)** |
| `skill23_studio/launch/teleop_allterrain.launch.py` | `src/skill23_studio/launch/teleop_allterrain.launch.py` **(NEW)** |
| `skill23_studio/config/hw_params/teleop_mecanum.yaml` | `src/skill23_studio/config/hw_params/teleop_mecanum.yaml` **(NEW)** |
| `skill23_studio/config/hw_params/teleop_xdrive.yaml` | `src/skill23_studio/config/hw_params/teleop_xdrive.yaml` **(NEW)** |
| `skill23_studio/config/hw_params/teleop_grip.yaml` | `src/skill23_studio/config/hw_params/teleop_grip.yaml` **(NEW)** |
| `skill23_studio/config/hw_params/teleop_allterrain.yaml` | `src/skill23_studio/config/hw_params/teleop_allterrain.yaml` **(NEW)** |
| `skill23_studio/setup.py` | `src/skill23_studio/setup.py` **(REPLACE)** — or only add the `simple_pose = ...` entry point line |

**Entry point to add** (if you prefer not to replace whole `setup.py`):

```python
'simple_pose = skill23_studio.simple_pose_node:main',
```

inside `console_scripts`.

### Laptop — `/home/tad/competition/workspaces/skill23_vision`

| This patch file | Copy to |
|---|---|
| `skill23_vision/teleop_capture.py` | `teleop_capture.py` **(NEW)** |
| `skill23_vision/start_teleop_capture.sh` | `start_teleop_capture.sh` **(NEW)** |
| `skill23_vision/requirements.txt` | merge / use if you need deps |
| `skill23_vision/README_LAPTOP.md` | optional reference |
| `skill23_vision/models/README.txt` | `models/README.txt` — put `best.pt` in `models/` |

**Do not** overwrite your existing Docker vision_gate / AprilTag packages.
These are additive scripts only.

---

## Build on Pi (after copy)

```bash
cd ~/skill23_ws   # or your competition path
source /opt/ros/humble/setup.bash
colcon build --packages-select skill23_studio
source install/setup.bash
```

---

## Run

### Pi (one robot)

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0    # must share topics with laptop over Wi‑Fi

# pick one:
ros2 launch skill23_studio teleop_mecanum.launch.py
ros2 launch skill23_studio teleop_xdrive.launch.py
ros2 launch skill23_studio teleop_grip.launch.py
ros2 launch skill23_studio teleop_allterrain.launch.py

# or:
ros2 launch skill23_studio teleop_arena.launch.py robot:=mecanum
```

Pose topics:

```text
/robot_pose          geometry_msgs/Pose2D     (x, y, theta)
/robot_pose_xyyaw    Float64MultiArray        [x, y, yaw]
```

Reset origin anytime:

```bash
ros2 service call /robot_pose/reset std_srvs/srv/Trigger {}
```

### Laptop

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
source /opt/ros/humble/setup.bash

cd /home/tad/competition/workspaces/skill23_vision
# once: pip3 install ultralytics opencv-python
# optional: pip3 install pupil_apriltags
# copy trained weights → models/best.pt

./start_teleop_capture.sh
```

| Button | Default index | Action |
|--------|---------------|--------|
| A | 0 | YOLO capture + save |
| B | 1 | AprilTag capture + save |

Saves under `captures/` (images + `detections.txt`).

---

## Geometry notes

| Robot | Drive node | Strafe (y) |
|-------|------------|------------|
| mecanum | `mecanum_node` | yes — mecanum wheel mix |
| xdrive | `xdrive_node` | yes — X-drive body_from_wheels |
| grip | `grip_node` | no — diff / skid |
| allterrain | `allterrain_node` | no — diff / skid |

`simple_pose` zeros at start and republishes the drive node's encoder+IMU
`/wheel/odom` as arena pose. Mecanum/xdrive teleop YAML maps left-stick X to
strafe (`axis_linear_y: 0`).

---

## Safety / Docker

- Laptop scripts live beside your competition `skill23_vision` — they do **not**
  change `vision_gate_node.py` or installed AprilTag packages.
- Pi changes are **new** launch/params/pose files + one `setup.py` entry.
- Existing Studio / commission / OMS flows are untouched.


---

## Camera live view

Pi sets `start_streams_on_init:=true` → Orbbec **starts streams** (live frames), not idle/toggle-off. Laptop shows continuous LIVE window; YOLO/AprilTag only on button/key.

## Change gamepad capture buttons

```bash
ros2 topic echo /joy          # see button indices
YOLO_BTN=2 TAG_BTN=3 ./start_teleop_capture.sh
# or: python3 teleop_capture.py --yolo-btn 2 --tag-btn 3
```

## Keyboard backup (laptop)

Copy also: `keyboard_teleop.py`, `start_keyboard_teleop.sh`  
Do **not** run joy_node together. Keys: W/S move, A/D strafe, Q/E turn, Y=YOLO, T=AprilTag, SPACE=stop.

## AprilTag — no download

Uses installed `pupil_apriltags` if present, else OpenCV AprilTag dict. Does **not** pip-install or download inside Docker.
