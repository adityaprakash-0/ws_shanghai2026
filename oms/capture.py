#!/usr/bin/env python3
"""
Capture color + depth images from an Orbbec Gemini E camera.

Requirements:
    pip install pyorbbecsdk opencv-python numpy

Controls (while the preview window is focused):
    s / SPACE  -> save the current color, depth (16-bit PNG) and colorized depth
    ESC / q    -> quit
"""

import os
import time

import cv2
import numpy as np
from pyorbbecsdk import (
    Config,
    OBError,
    OBFormat,
    OBSensorType,
    Pipeline,
)

SAVE_DIR = "captures"
COLOR_RES = (640, 480, 30)   # width, height, fps  (set to None to use default)
DEPTH_RES = (640, 480, 30)


def frame_to_bgr(color_frame):
    """Convert an Orbbec color frame to an OpenCV BGR image."""
    w, h = color_frame.get_width(), color_frame.get_height()
    fmt = color_frame.get_format()
    data = np.asanyarray(color_frame.get_data())

    if fmt == OBFormat.MJPG:
        return cv2.imdecode(data, cv2.IMREAD_COLOR)
    if fmt == OBFormat.RGB:
        return cv2.cvtColor(data.reshape(h, w, 3), cv2.COLOR_RGB2BGR)
    if fmt == OBFormat.BGR:
        return data.reshape(h, w, 3)
    if fmt == OBFormat.YUYV:
        return cv2.cvtColor(data.reshape(h, w, 2), cv2.COLOR_YUV2BGR_YUYV)
    if fmt == OBFormat.UYVY:
        return cv2.cvtColor(data.reshape(h, w, 2), cv2.COLOR_YUV2BGR_UYVY)
    if fmt == OBFormat.NV12:
        return cv2.cvtColor(data.reshape(h * 3 // 2, w), cv2.COLOR_YUV2BGR_NV12)
    if fmt == OBFormat.NV21:
        return cv2.cvtColor(data.reshape(h * 3 // 2, w), cv2.COLOR_YUV2BGR_NV21)
    print(f"Unsupported color format: {fmt}")
    return None


def depth_to_mm(depth_frame):
    """Return the depth frame as a float32 array in millimetres."""
    w, h = depth_frame.get_width(), depth_frame.get_height()
    scale = depth_frame.get_depth_scale()          # units -> mm
    raw = np.frombuffer(depth_frame.get_data(), dtype=np.uint16).reshape(h, w)
    return raw.astype(np.float32) * scale


def colorize(depth_mm, min_mm=200.0, max_mm=4000.0):
    """Make a viewable false-colour image from a depth map."""
    clipped = np.clip(depth_mm, min_mm, max_mm)
    norm = ((clipped - min_mm) / (max_mm - min_mm) * 255).astype(np.uint8)
    colored = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
    colored[depth_mm == 0] = 0                     # black out invalid pixels
    return colored


def pick_profile(pipeline, sensor_type, res, fmt=None):
    """Pick a stream profile, falling back to the device default."""
    profiles = pipeline.get_stream_profile_list(sensor_type)
    if res is not None:
        try:
            return profiles.get_video_stream_profile(res[0], res[1], fmt, res[2])
        except OBError:
            print(f"{sensor_type} {res} not available, using default profile.")
    return profiles.get_default_video_stream_profile()


def main():
    os.makedirs(SAVE_DIR, exist_ok=True)

    pipeline = Pipeline()
    config = Config()

    color_profile = pick_profile(
        pipeline, OBSensorType.COLOR_SENSOR, COLOR_RES, OBFormat.RGB
    )
    depth_profile = pick_profile(
        pipeline, OBSensorType.DEPTH_SENSOR, DEPTH_RES, OBFormat.Y16
    )
    config.enable_stream(color_profile)
    config.enable_stream(depth_profile)

    print("Color:", color_profile)
    print("Depth:", depth_profile)

    pipeline.start(config)
    print("Streaming. Press 's' to save, 'q' or ESC to quit.")

    saved = 0
    try:
        while True:
            frames = pipeline.wait_for_frames(100)
            if frames is None:
                continue

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if color_frame is None or depth_frame is None:
                continue

            color_img = frame_to_bgr(color_frame)
            depth_mm = depth_to_mm(depth_frame)
            if color_img is None:
                continue

            depth_vis = colorize(depth_mm)
            preview = np.hstack(
                [color_img, cv2.resize(depth_vis, color_img.shape[1::-1])]
            )
            cv2.imshow("Gemini E  |  color (left) - depth (right)", preview)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("s"), ord(" ")):
                stamp = time.strftime("%Y%m%d_%H%M%S")
                cv2.imwrite(f"{SAVE_DIR}/color_{stamp}.png", color_img)
                cv2.imwrite(f"{SAVE_DIR}/depth_{stamp}.png",
                            depth_mm.astype(np.uint16))          # 16-bit, mm
                cv2.imwrite(f"{SAVE_DIR}/depth_vis_{stamp}.png", depth_vis)
                saved += 1
                print(f"Saved frame {saved} ({stamp})  "
                      f"center depth = {depth_mm[depth_mm.shape[0] // 2, depth_mm.shape[1] // 2]:.0f} mm")
            elif key in (27, ord("q")):
                break
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print(f"Done. {saved} frame(s) written to ./{SAVE_DIR}/")


if __name__ == "__main__":
    main()