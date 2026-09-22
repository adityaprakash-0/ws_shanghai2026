# skill23_vision — laptop teleop capture (YOLO + AprilTag)

Standalone scripts for the **laptop**. Does **not** modify the Pi Docker
`skill23_vision` vision_gate package.

## What it does

1. Subscribes to Pi camera (`/camera/color/image_raw[/compressed]`)
2. Subscribes to `/robot_pose` (x, y, yaw from Pi simple_pose)
3. Subscribes to `/joy` (gamepad on this laptop)
4. **Button A (0)** → take picture → YOLO → show boxes + class + robot pose + timestamp → save image + line in `captures/detections.txt`
5. **Button B (1)** → take picture → AprilTag → show id + robot pose + timestamp → save image + line in txt

Detection runs **only** on button press.

## Setup (once)

```bash
export ROS_DOMAIN_ID=0          # same as Pi
export ROS_LOCALHOST_ONLY=0     # must see Pi over Wi‑Fi
source /opt/ros/humble/setup.bash

cd /home/tad/competition/workspaces/skill23_vision   # or this folder
pip3 install -r requirements.txt
pip3 install ultralytics          # YOLO
# optional better AprilTag:
# pip3 install pupil_apriltags

# put weights here:
mkdir -p models
# copy best.pt → models/best.pt
```

AprilTag: uses `pupil_apriltags` if installed, otherwise OpenCV
`DICT_APRILTAG_36h11` (no extra package). Existing AprilTag installs in your
competition Docker/workspace are left untouched.

## Run

**Terminal 1 — Pi** (already running teleop launch):

```bash
ros2 launch skill23_studio teleop_mecanum.launch.py   # or xdrive/grip/allterrain
```

**Terminal 2 — laptop**:

```bash
./start_teleop_capture.sh
# or:
ros2 run joy joy_node
python3 teleop_capture.py
```

## Buttons

| Button | Index | Action |
|--------|-------|--------|
| A | 0 | YOLO capture |
| B | 1 | AprilTag capture |

Override: `YOLO_BTN=2 TAG_BTN=3 ./start_teleop_capture.sh`

## Output

```
captures/
  yolo_YYYYMMDD_*.jpg
  apriltag_YYYYMMDD_*.jpg
  detections.txt
```

Each txt line has timestamp, robot pose (x,y,yaw), object/tag id, image coords.
