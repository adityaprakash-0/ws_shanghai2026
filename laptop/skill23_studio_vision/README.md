# Skill23 Studio Vision (laptop)

Copy this folder to the **laptop**. Heavy vision (ArUco, YOLO, OCR, barcode, QR, color) runs here — **not on the Pi**.

The Pi only:
1. turns the camera on for that waypoint
2. sends a start request
3. waits for ACK / result / done

Frames come **straight from the Orbbec driver** — the Pi does not relay or
re-encode them, and the dashboard shows no preview:

| Topic | Used for |
|---|---|
| `/camera/color/image_raw/compressed` | preferred colour source |
| `/camera/color/image_raw` | fallback colour source (MJPG/RGB/YUYV) |
| `/camera/depth/image_raw` | depth, only for `*_depth` actions |

The hub subscribes only while a job is running, so it costs nothing while idle.

## One-time setup

```bash
# same ROS_DOMAIN_ID as the Pi (default 0)
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash

cd skill23_studio_vision
python3 -m pip install -r requirements.txt
# optional extras:
#   pip install ultralytics pytesseract pyzbar
```

Put a trained YOLO weight at `models/best.pt` (or set `YOLO_MODEL=/path/to.pt`).

## Launch

```bash
./start.sh
```

Pick one detector or `all`. Nothing processes frames until the Pi sends `/studio/vision/request`.

## Handshake

| Topic | Who | Payload |
|---|---|---|
| `/studio/vision/request` | Pi → laptop | `{id, type, timeout_sec, params}` |
| `/studio/vision/ack` | laptop → Pi | `{id, status:"started"}` |
| `/studio/vision/result` | laptop → Pi | `{id, ok, message, ...}` |
| `/studio/vision/done` | Pi → laptop | `{id}` — laptop stops that detector |

If the laptop does not ACK within ~1.5s the Pi retries 4 times, then skips to the next waypoint.

## Depth approach (dashboard)

Pick e.g. **ArUco align + depth approach**. Field **Closer by (cm)**:

- camera depth = 20 cm, you want to stop at 15 cm → enter **5**
- or set **stop at depth (cm)** = 15 (absolute standoff; 0 means use closer-by)

Laptop aligns on the target, then drives forward using live depth until that standoff. Files:

- `detectors/aruco_depth.py`
- `detectors/color_depth.py`
- `detectors/yolo_depth.py`
- `detectors/qr_depth.py`
- `detectors/barcode_depth.py`
- `detectors/ocr_depth.py`
- `detectors/depth_util.py`
