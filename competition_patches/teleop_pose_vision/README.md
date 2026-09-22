# Competition patch — arena teleop pose + YOLO / AprilTag capture

Offline kit for competition (no internet). Copy files onto the **Pi** and **laptop**
as listed below. Does **not** modify Pi Docker `skill23_vision` vision_gate.

---

## Quick answers to common doubts

### 1) Does the camera start with live frames, or idle / toggle-off?

**Live frames ON.** Launch sets:

- `enable_color:=true`
- `enable_depth:=true`
- `start_streams_on_init:=true`

Orbbec then calls `startStreams()` → `/camera/color/image_raw` (and compressed)
publish continuously while you drive. Laptop shows a **LIVE** window. YOLO /
AprilTag run **only** when you press the capture button (or Y / T on keyboard).

### 2) Does gamepad.enabled actually move the robot?

**Yes.** Flow:

```text
laptop joy_node → /joy
    → Pi GamepadController (inside control_server)
    → /cmd_vel
    → mecanum_node / xdrive_node / grip_node / allterrain_node
    → Titan motors
```

`teleop_*.yaml` has `gamepad.enabled: true`.

### 3) Which sticks / buttons drive the robot? (Pi gamepad)

**Mecanum / XDrive** (`teleop_mecanum.yaml` / `teleop_xdrive.yaml`):

| Stick / button | Joy index | Effect |
|----------------|-----------|--------|
| Left stick Y | axis **1** | forward / back (`linear.x`) |
| Left stick X | axis **0** | strafe (`linear.y`) |
| Right stick X | axis **3** | rotate (`angular.z`) |
| RB | button **5** | turbo (2×) |

**Grip / Allterrain** (no strafe): axis 1 = forward, axis 2 = turn, axis_linear_y = **-1** (off), RB = turbo.

### 4) Which buttons are YOLO / AprilTag? (laptop only)

| Button | Default index | Action |
|--------|---------------|--------|
| **A** | 0 | YOLO capture + save |
| **B** | 1 | AprilTag capture + save |

These do **not** move the robot. Only laptop `teleop_capture` listens to them.

### 5) How do I change button numbers if A/B are wrong?

```bash
ros2 topic echo /joy          # press buttons, note indices
YOLO_BTN=2 TAG_BTN=3 ./start_teleop_capture.sh
# or:
python3 teleop_capture.py --yolo-btn 2 --tag-btn 3
```

### 6) Which YOLO `best.pt` is used?

Lookup order in `teleop_capture.py`:

1. `$YOLO_MODEL` if set  
2. else first of `models/best.pt`, `models/yolo.pt`, `models/yolov8n.pt`  
3. else expects `models/best.pt` (error if missing)

After you train in skill23_vision, copy weights to:

```text
skill23_vision/models/best.pt
```

or:

```bash
export YOLO_MODEL=/path/to/your_trained.pt
```

### 7) Will AprilTag download packages inside Docker?

**No.** Nothing is pip-installed or downloaded at runtime.

1. Uses `pupil_apriltags` **if already installed** in your image  
2. Else OpenCV built-in AprilTag / ArUco dict  
3. If neither → clear error (no download)

Startup logs: `AprilTag backend: ...`

### 8) What does `export ROS_LOCALHOST_ONLY=0` do?

Lets the Pi and laptop share ROS topics over Wi‑Fi (`/joy`, `/camera/...`, `/robot_pose`).  
If left at `1`, laptop cannot see the Pi.

Also use the same `ROS_DOMAIN_ID` on both machines (default `0`).

### 9) Keyboard backup if gamepad fails?

```bash
./start_keyboard_teleop.sh
```

Do **not** run `joy_node` at the same time (both publish `/cmd_vel`).

| Key | Action |
|-----|--------|
| W / S | forward / back |
| A / D | strafe |
| Q / E | rotate |
| SPACE | stop |
| Y | YOLO |
| T | AprilTag |
| ESC / X | quit |

### 10) If I run `joy_node` with sudo, will `teleop_capture` see `/joy`?

**Often no** if teleop runs as a normal user. Fast DDS shared memory breaks
across **root ↔ user**, so sudo `joy_node` and user `teleop_capture` may not
discover each other.

**Preferred (no sudo on laptop):**

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/humble/setup.bash
ros2 run joy joy_node                    # NOT sudo
# other terminal:
python3 teleop_capture.py                # NOT sudo
# or simply:
./start_teleop_capture.sh
```

Correct ROS 2 command is `ros2 run joy joy_node` (not `ros run joy_node joy_node`).

**If you insist on sudo for joy:** run **both** as root with the **same** env
and force UDP:

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/humble/setup.bash

sudo -E env PATH=$PATH PYTHONPATH=$PYTHONPATH LD_LIBRARY_PATH=$LD_LIBRARY_PATH \
  FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
  ROS_LOCALHOST_ONLY=0 \
  ros2 run joy joy_node

# other terminal — also sudo (same env):
sudo -E env PATH=$PATH PYTHONPATH=$PYTHONPATH LD_LIBRARY_PATH=$LD_LIBRARY_PATH \
  FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
  ROS_LOCALHOST_ONLY=0 \
  python3 teleop_capture.py
```

OpenCV windows / display can be worse under sudo. Prefer no sudo on the laptop.
Pi `control_server` already gets UDPv4 from the teleop launch so it can see
laptop `/joy` when joy runs as a normal user.

---

## What to copy where

### A) Pi — into your `skill23_ws` (or competition studio workspace)

| File in this patch | Destination |
|---|---|
| `skill23_studio/skill23_studio/simple_pose_node.py` | `src/skill23_studio/skill23_studio/simple_pose_node.py` **NEW** |
| `skill23_studio/launch/teleop_arena.launch.py` | `src/skill23_studio/launch/teleop_arena.launch.py` **NEW** |
| `skill23_studio/launch/teleop_mecanum.launch.py` | `src/skill23_studio/launch/teleop_mecanum.launch.py` **NEW** |
| `skill23_studio/launch/teleop_xdrive.launch.py` | `src/skill23_studio/launch/teleop_xdrive.launch.py` **NEW** |
| `skill23_studio/launch/teleop_grip.launch.py` | `src/skill23_studio/launch/teleop_grip.launch.py` **NEW** |
| `skill23_studio/launch/teleop_allterrain.launch.py` | `src/skill23_studio/launch/teleop_allterrain.launch.py` **NEW** |
| `skill23_studio/config/hw_params/teleop_mecanum.yaml` | `src/skill23_studio/config/hw_params/teleop_mecanum.yaml` **NEW** |
| `skill23_studio/config/hw_params/teleop_xdrive.yaml` | `src/skill23_studio/config/hw_params/teleop_xdrive.yaml` **NEW** |
| `skill23_studio/config/hw_params/teleop_grip.yaml` | `src/skill23_studio/config/hw_params/teleop_grip.yaml` **NEW** |
| `skill23_studio/config/hw_params/teleop_allterrain.yaml` | `src/skill23_studio/config/hw_params/teleop_allterrain.yaml` **NEW** |
| `skill23_studio/setup.py` | `src/skill23_studio/setup.py` **REPLACE** (or only add the entry point below) |

**Minimal setup.py change** (if you do not replace the whole file):

```python
'simple_pose = skill23_studio.simple_pose_node:main',
```

inside `console_scripts`.

Then on Pi:

```bash
cd ~/skill23_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select skill23_studio
source install/setup.bash
```

### B) Laptop — `/home/tad/competition/workspaces/skill23_vision`

| File in this patch | Destination |
|---|---|
| `skill23_vision/teleop_capture.py` | `teleop_capture.py` **NEW** |
| `skill23_vision/keyboard_teleop.py` | `keyboard_teleop.py` **NEW** |
| `skill23_vision/start_teleop_capture.sh` | `start_teleop_capture.sh` **NEW** |
| `skill23_vision/start_keyboard_teleop.sh` | `start_keyboard_teleop.sh` **NEW** |
| `skill23_vision/requirements.txt` | optional merge |
| `skill23_vision/models/README.txt` | `models/README.txt` — put `best.pt` here |

Do **not** overwrite Docker vision_gate / existing AprilTag installs. These are additive.

---

## What each file does

### Pi — pose + launch

| File | Role |
|------|------|
| **`simple_pose_node.py`** | Subscribes `/wheel/odom` (encoders + IMU from drive node). Zeros origin at start → `(0,0,0)`. Publishes `/robot_pose` (`Pose2D`) and `/robot_pose_xyyaw` `[x,y,yaw]`. Reset: `ros2 service call /robot_pose/reset std_srvs/srv/Trigger {}`. Mecanum/xdrive keep strafe `y`; grip/allterrain are diff. |
| **`teleop_arena.launch.py`** | Main launch. Starts in order: (1) `control_server` sudo with gamepad ON + Fast DDS UDPv4 so laptop `/joy` works, (2) drive node, (3) `simple_pose`, (4) Orbbec Gemini E with streams ON. Forces `ROS_LOCALHOST_ONLY=0`. |
| **`teleop_mecanum.launch.py`** | Thin wrapper → arena with `robot:=mecanum` (strafe ON). |
| **`teleop_xdrive.launch.py`** | Thin wrapper → `robot:=xdrive` (strafe ON). |
| **`teleop_grip.launch.py`** | Thin wrapper → `robot:=grip` (no strafe). |
| **`teleop_allterrain.launch.py`** | Thin wrapper → `robot:=allterrain` (no strafe). |
| **`teleop_mecanum.yaml`** | HW params: Titans, IMU, **gamepad enabled**, strafe axis 0, turn axis 3, turbo btn 5, scale 0.2. |
| **`teleop_xdrive.yaml`** | Same idea for xdrive (strafe ON). |
| **`teleop_grip.yaml`** | Gamepad ON, no strafe (`axis_linear_y: -1`). |
| **`teleop_allterrain.yaml`** | Gamepad ON, no strafe. |
| **`setup.py`** | Registers executable `simple_pose` and installs launch/config. |

### Laptop — capture + keyboard

| File | Role |
|------|------|
| **`teleop_capture.py`** | Subscribes camera + `/robot_pose` + `/joy`. Shows continuous **LIVE** window. On YOLO button: grab frame → YOLO → boxes/class/pose/time → save image + `captures/detections.txt`. On AprilTag button: same for tags. Detection **only** on button press. |
| **`start_teleop_capture.sh`** | Starts `joy_node` + `teleop_capture.py`. Env: `YOLO_BTN`, `TAG_BTN`, `OUT`, `ROS_DOMAIN_ID`. |
| **`keyboard_teleop.py`** | Backup: publishes `/cmd_vel` from keyboard + live view + Y/T capture. Same save format. |
| **`start_keyboard_teleop.sh`** | Runs keyboard teleop **without** joy_node. |
| **`requirements.txt`** | Optional notes (`opencv`, `ultralytics`, `pupil_apriltags`). Install offline beforehand if needed. |
| **`models/`** | Put trained `best.pt` here after training. |

---

## How to run (full stack)

### Pi

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
source /opt/ros/humble/setup.bash
source ~/skill23_ws/install/setup.bash

# pick your robot:
ros2 launch skill23_studio teleop_mecanum.launch.py
# ros2 launch skill23_studio teleop_xdrive.launch.py
# ros2 launch skill23_studio teleop_grip.launch.py
# ros2 launch skill23_studio teleop_allterrain.launch.py
```

Launch order inside:

1. **control_server** — HW + gamepad (`/joy` → `/cmd_vel`)  
2. **drive node** — motors + `/wheel/odom`  
3. **simple_pose** — `/robot_pose` from `(0,0,0)`  
4. **Orbbec camera** — live color stream  

### Laptop — gamepad mode

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
cd /home/tad/competition/workspaces/skill23_vision
# once (offline): ensure ultralytics + opencv present; put models/best.pt
./start_teleop_capture.sh
```

Drive with sticks; press **A** for YOLO, **B** for AprilTag.

### Laptop — keyboard backup

```bash
# do NOT run start_teleop_capture.sh (no joy_node)
./start_keyboard_teleop.sh
# focus the OpenCV window, then W/A/S/D/Q/E, Y, T
```

---

## Topics cheat-sheet

| Topic | Who | Meaning |
|-------|-----|---------|
| `/joy` | laptop joy_node | raw gamepad |
| `/cmd_vel` | Pi gamepad (or laptop keyboard) | velocity to motors |
| `/wheel/odom` | drive node | encoder + IMU odom |
| `/robot_pose` | simple_pose | arena x, y, yaw from start |
| `/robot_pose_xyyaw` | simple_pose | `[x, y, yaw]` |
| `/camera/color/image_raw` | Orbbec | live color |
| `/camera/color/image_raw/compressed` | Orbbec | preferred by laptop |

---

## Output on laptop

```text
captures/
  yolo_YYYYMMDD_*.jpg        # annotated image
  apriltag_YYYYMMDD_*.jpg
  detections.txt             # timestamp, robot pose, object/tag, image name
```

Each detection line includes: timestamp, robot pose `(x,y,yaw)`, object name or tag id, image coords, image filename.

---

## Safety / Docker

- Laptop scripts are **additive** — they do not change `vision_gate_node.py` or installed AprilTag packages.  
- Pi changes are **new** launch/params/pose files + one `setup.py` entry.  
- Studio / commission / OMS flows are untouched.  
- No runtime `pip install` / model download from these scripts.
