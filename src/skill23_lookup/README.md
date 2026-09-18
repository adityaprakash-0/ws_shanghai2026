# skill23_lookup

Waypoint **Lookup OCR** / **Lookup ArUco** for Skill23 Studio Nav2 missions.

> Note: intended path was `~/skill23_vision`, but that filesystem may be read-only
> in some agent sandboxes — this package lives in `skill23_ws/src/skill23_lookup`
> and builds with the main workspace. Symlink into `~/skill23_vision/src` if desired.

## Offline models

```bash
ros2 run skill23_lookup prefetch_models
# or: python3 -m skill23_lookup.prefetch_models
```

EasyOCR weights: `~/.EasyOCR/model/craft_mlt_25k.pth`, `english_g2.pth`.
ArUco uses OpenCV dictionaries (no download).

## Topics

| Topic | Type | Role |
|-------|------|------|
| `/studio/lookup/cmd` | `std_msgs/String` JSON | start lookup |
| `/studio/lookup/result` | `std_msgs/String` JSON | success + `next_wp` |
| `/studio/lookup/status` | `std_msgs/String` JSON | live status |
| `/studio/lookup/cancel` | `std_msgs/Empty` | abort |
| `/studio/camera/action_need` | `std_msgs/Bool` | ask Studio to toggle color stream |

Cmd example: `{"cmd_id":"…","mode":"ocr","confirm_frames":3,"retries":3}`  
Result: `{"success":true,"next_wp":4,"mode":"ocr","message":"…"}` — `next_wp` is **1-based** WP index.
