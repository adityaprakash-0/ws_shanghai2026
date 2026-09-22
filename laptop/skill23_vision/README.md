# skill23_vision — laptop teleop capture (YOLO + AprilTag)

Standalone scripts for the **laptop**. Does **not** modify the Pi Docker
`skill23_vision` vision_gate package. **Nothing is downloaded at runtime.**

## Camera live view

Pi launch uses `start_streams_on_init:=true` → Orbbec calls `startStreams()`.
Color frames publish continuously on `/camera/color/image_raw` (not idle /
toggle-off). Laptop shows a **LIVE** window while you drive; YOLO/AprilTag
run **only** when you press the capture button/key.

## Gamepad mode

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
./start_teleop_capture.sh
```

| Button | Default index | Action |
|--------|---------------|--------|
| A | 0 | YOLO capture + save |
| B | 1 | AprilTag capture + save |

### Change button numbers

```bash
# find your indices:
ros2 topic echo /joy
# then:
YOLO_BTN=2 TAG_BTN=3 ./start_teleop_capture.sh
# or:
python3 teleop_capture.py --yolo-btn 2 --tag-btn 3
```

## Keyboard backup (no gamepad)

Do **not** run `joy_node` at the same time (fights `/cmd_vel`).

```bash
./start_keyboard_teleop.sh
# focus the OpenCV window
```

| Key | Action |
|-----|--------|
| W / S | forward / back |
| A / D | strafe left / right |
| Q / E | rotate |
| SPACE | stop |
| Y | YOLO capture |
| T | AprilTag capture |
| ESC / X | quit |

## YOLO weights

Put trained weights at `models/best.pt`, or:

```bash
export YOLO_MODEL=/path/to/your.pt
```

## AprilTag — no download in Docker

At startup the node logs the backend:

1. **`pupil_apriltags`** if already installed in the image/env
2. else **OpenCV** `DICT_APRILTAG_36h11` / aruco (bundled with opencv — no pip)

It never calls `pip install` and never downloads models. If neither backend
exists, AprilTag button fails with a clear error (YOLO still works if
`ultralytics` + `best.pt` are present offline).

## Output

```
captures/
  yolo_*.jpg
  apriltag_*.jpg
  detections.txt
```
