#!/usr/bin/env python3
"""Laptop teleop capture — YOLO + AprilTag on gamepad buttons.

Runs on the laptop (same ROS_DOMAIN_ID as the Pi). Gamepad is plugged into
the laptop; joy_node publishes /joy (Pi gamepad component drives the robot).

Default buttons (Xbox/Logitech, change with params):
  A (button 0)  → YOLO: grab frame, detect, show + save
  B (button 1)  → AprilTag: grab frame, detect, show + save

Only runs detection when you press the button (camera can stream continuously).

Saves under ./captures/ (images + detections.txt).
"""

from __future__ import annotations

import argparse
import math
import os
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image, Joy
from std_msgs.msg import Float64MultiArray


# ---------------------------------------------------------------------------
# Image helpers
# ---------------------------------------------------------------------------

def _color_to_bgr(msg: Image) -> Optional[np.ndarray]:
    enc = (msg.encoding or '').lower().replace('-', '').replace('_', '')
    data = np.frombuffer(msg.data, dtype=np.uint8)
    if enc in ('jpeg', 'jpg', 'mjpeg', 'mjpg'):
        return cv2.imdecode(data, cv2.IMREAD_COLOR)
    h, w = int(msg.height), int(msg.width)
    if h <= 0 or w <= 0 or data.size == 0:
        return None
    try:
        if enc in ('bgr8', '8uc3'):
            return data.reshape((h, w, 3)).copy()
        if enc == 'rgb8':
            return cv2.cvtColor(data.reshape((h, w, 3)), cv2.COLOR_RGB2BGR)
        if enc in ('mono8', '8uc1'):
            return cv2.cvtColor(data.reshape((h, w)), cv2.COLOR_GRAY2BGR)
        if enc in ('yuv422', 'yuyv', 'yuy2', 'yuyv422'):
            return cv2.cvtColor(data.reshape((h, w, 2)), cv2.COLOR_YUV2BGR_YUY2)
    except ValueError:
        return None
    return None


def _compressed_to_bgr(msg: CompressedImage) -> Optional[np.ndarray]:
    return cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)


# ---------------------------------------------------------------------------
# Detectors (self-contained — do not touch Docker vision_gate)
# ---------------------------------------------------------------------------

_YOLO = None
_YOLO_ERR = ''


def _yolo_path() -> Path:
    env = os.environ.get('YOLO_MODEL', '')
    if env:
        return Path(env).expanduser()
    here = Path(__file__).resolve().parent / 'models'
    for name in ('best.pt', 'yolo.pt', 'yolov8n.pt'):
        p = here / name
        if p.is_file():
            return p
    return here / 'best.pt'


def run_yolo(frame: np.ndarray, conf: float = 0.4) -> Dict[str, Any]:
    global _YOLO, _YOLO_ERR
    if _YOLO is None and not _YOLO_ERR:
        path = _yolo_path()
        if not path.is_file():
            _YOLO_ERR = f'YOLO model missing: {path}'
        else:
            try:
                from ultralytics import YOLO
                _YOLO = YOLO(str(path))
            except Exception as exc:
                _YOLO_ERR = f'ultralytics load failed: {exc}'
    if _YOLO is None:
        return {'ok': False, 'hits': [], 'message': _YOLO_ERR or 'yolo unavailable'}

    res = _YOLO.predict(frame, verbose=False, conf=conf)[0]
    names = res.names or {}
    hits = []
    if res.boxes is not None:
        for b in res.boxes:
            cls_id = int(b.cls[0])
            name = str(names.get(cls_id, cls_id))
            score = float(b.conf[0])
            xyxy = [float(v) for v in b.xyxy[0].tolist()]
            cx = (xyxy[0] + xyxy[2]) / 2.0
            cy = (xyxy[1] + xyxy[3]) / 2.0
            hits.append({
                'class': name, 'conf': round(score, 3),
                'box': xyxy, 'cx': cx, 'cy': cy,
            })
    if not hits:
        return {'ok': False, 'hits': [], 'message': 'no yolo detections'}
    return {'ok': True, 'hits': hits, 'message': f'{len(hits)} objects'}


def run_apriltag(frame: np.ndarray) -> Dict[str, Any]:
    """Prefer pupil_apriltags; fall back to OpenCV ArUco AprilTag dict."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hits: List[Dict[str, Any]] = []

    try:
        from pupil_apriltags import Detector
        det = getattr(run_apriltag, '_det', None)
        if det is None:
            det = Detector(families='tag36h11')
            run_apriltag._det = det  # type: ignore[attr-defined]
        for t in det.detect(gray):
            c = t.corners.astype(np.float32)
            cx, cy = float(t.center[0]), float(t.center[1])
            hits.append({
                'id': int(t.tag_id),
                'cx': cx, 'cy': cy,
                'corners': c.tolist(),
            })
    except Exception:
        # OpenCV AprilTag dictionaries (no extra pip package)
        dict_id = None
        for name in ('DICT_APRILTAG_36h11', 'DICT_APRILTAG_25h9', 'DICT_4X4_50'):
            if hasattr(cv2.aruco, name):
                dict_id = getattr(cv2.aruco, name)
                break
        if dict_id is None:
            return {'ok': False, 'hits': [], 'message': 'no apriltag backend'}
        dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
        try:
            params = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(dictionary, params)
            corners, ids, _ = detector.detectMarkers(gray)
        except Exception:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
        if ids is not None:
            for i, mid in enumerate(ids.flatten().tolist()):
                c = corners[i][0]
                hits.append({
                    'id': int(mid),
                    'cx': float(c[:, 0].mean()),
                    'cy': float(c[:, 1].mean()),
                    'corners': c.tolist(),
                })

    if not hits:
        return {'ok': False, 'hits': [], 'message': 'no apriltags'}
    return {'ok': True, 'hits': hits, 'message': f'{len(hits)} tags'}


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

def apriltag_backend_info() -> str:
    """Report which AprilTag backend will be used — never downloads anything."""
    try:
        import pupil_apriltags  # noqa: F401
        return 'pupil_apriltags (already installed)'
    except Exception:
        pass
    if hasattr(cv2, 'aruco') and (
            hasattr(cv2.aruco, 'DICT_APRILTAG_36h11')
            or hasattr(cv2.aruco, 'DICT_4X4_50')):
        return 'OpenCV aruco AprilTag dict (no download)'
    return 'NONE — install opencv with aruco or pupil_apriltags offline'


class TeleopCaptureNode(Node):
    def __init__(
            self,
            *,
            out_dir: Path,
            yolo_btn: int,
            tag_btn: int,
            conf: float,
            live_view: bool = True) -> None:
        super().__init__('teleop_capture')
        self._out = out_dir
        self._out.mkdir(parents=True, exist_ok=True)
        self._txt = self._out / 'detections.txt'
        self._yolo_btn = yolo_btn
        self._tag_btn = tag_btn
        self._conf = conf
        self._live_view = live_view

        self._frame: Optional[np.ndarray] = None
        self._pose = (0.0, 0.0, 0.0)
        self._prev_buttons: List[int] = []
        self._busy = False

        self.create_subscription(
            CompressedImage,
            '/camera/color/image_raw/compressed',
            self._on_compressed,
            qos_profile_sensor_data)
        self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self._on_image,
            qos_profile_sensor_data)
        self.create_subscription(Pose2D, '/robot_pose', self._on_pose, 10)
        self.create_subscription(
            Float64MultiArray, '/robot_pose_xyyaw', self._on_xyyaw, 10)
        self.create_subscription(Joy, '/joy', self._on_joy, 10)

        if self._live_view:
            # Continuous live preview while driving (~10 Hz)
            self.create_timer(0.1, self._show_live)

        self.get_logger().info(
            f'teleop_capture ready  YOLO btn={yolo_btn}  AprilTag btn={tag_btn}  '
            f'live_view={live_view}  out={self._out}')
        self.get_logger().info(f'AprilTag backend: {apriltag_backend_info()}')
        self.get_logger().info(f'YOLO weights: {_yolo_path()}')

    def _on_compressed(self, msg: CompressedImage) -> None:
        frame = _compressed_to_bgr(msg)
        if frame is not None:
            self._frame = frame

    def _on_image(self, msg: Image) -> None:
        if self._frame is not None:
            return  # prefer compressed
        frame = _color_to_bgr(msg)
        if frame is not None:
            self._frame = frame

    def _on_pose(self, msg: Pose2D) -> None:
        self._pose = (float(msg.x), float(msg.y), float(msg.theta))

    def _on_xyyaw(self, msg: Float64MultiArray) -> None:
        if len(msg.data) >= 3:
            self._pose = (float(msg.data[0]), float(msg.data[1]), float(msg.data[2]))

    def _on_joy(self, msg: Joy) -> None:
        buttons = list(msg.buttons)
        prev = self._prev_buttons
        self._prev_buttons = buttons

        def pressed(idx: int) -> bool:
            if idx < 0 or idx >= len(buttons):
                return False
            was = prev[idx] if idx < len(prev) else 0
            return buttons[idx] == 1 and was == 0

        if self._busy:
            return
        if pressed(self._yolo_btn):
            self.get_logger().info(
                f'[BUTTON] YOLO pressed (joy button {self._yolo_btn}) — capturing…')
            print(f'[BUTTON] YOLO pressed (joy button {self._yolo_btn}) — capturing…',
                  flush=True)
            self._busy = True
            try:
                self._capture_yolo()
            finally:
                self._busy = False
        elif pressed(self._tag_btn):
            self.get_logger().info(
                f'[BUTTON] AprilTag pressed (joy button {self._tag_btn}) — capturing…')
            print(f'[BUTTON] AprilTag pressed (joy button {self._tag_btn}) — capturing…',
                  flush=True)
            self._busy = True
            try:
                self._capture_apriltag()
            finally:
                self._busy = False

    def _stamp(self) -> str:
        return datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]

    def _pose_str(self) -> str:
        x, y, yaw = self._pose
        return f'x={x:.3f} y={y:.3f} yaw={yaw:.3f}'

    def _append_txt(self, line: str) -> None:
        with open(self._txt, 'a', encoding='utf-8') as f:
            f.write(line.rstrip() + '\n')

    def _show(self, title: str, img: np.ndarray) -> None:
        cv2.imshow(title, img)
        cv2.waitKey(1)

    def _show_live(self) -> None:
        """Always-on live camera window (does not run YOLO/AprilTag)."""
        if self._frame is None:
            return
        vis = self._frame.copy()
        x, y, yaw = self._pose
        cv2.putText(
            vis,
            f'LIVE  pose x={x:.2f} y={y:.2f} yaw={yaw:.2f}  '
            f'YOLO=btn{self._yolo_btn} TAG=btn{self._tag_btn}',
            (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        cv2.imshow('teleop live', vis)
        cv2.waitKey(1)

    def _capture_yolo(self) -> None:
        frame = None if self._frame is None else self._frame.copy()
        if frame is None:
            self.get_logger().warn('YOLO: no camera frame yet')
            return
        ts = self._stamp()
        pose = self._pose_str()
        result = run_yolo(frame, conf=self._conf)
        vis = frame.copy()
        lines = [f'[YOLO] {ts} robot_pose {pose}']

        if result['ok']:
            for h in result['hits']:
                x1, y1, x2, y2 = [int(v) for v in h['box']]
                label = f"{h['class']} {h['conf']:.2f}"
                cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 200, 0), 2)
                cv2.putText(
                    vis, label, (x1, max(20, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 0), 2)
                # Object image coords + robot pose as rough world hint
                lines.append(
                    f"  object={h['class']} conf={h['conf']} "
                    f"box=[{x1},{y1},{x2},{y2}] "
                    f"img_xy=({h['cx']:.1f},{h['cy']:.1f}) "
                    f"robot_pose=({self._pose[0]:.3f},{self._pose[1]:.3f},"
                    f"{self._pose[2]:.3f})")
        else:
            lines.append(f"  {result['message']}")

        cv2.putText(
            vis, f'YOLO  {pose}  {ts}', (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

        img_path = self._out / f'yolo_{ts}.jpg'
        cv2.imwrite(str(img_path), vis)
        for line in lines:
            self._append_txt(line)
        self._append_txt(f'  image={img_path.name}')
        self.get_logger().info(f'YOLO saved {img_path.name} — {result["message"]}')
        self._show('YOLO capture', vis)

    def _capture_apriltag(self) -> None:
        frame = None if self._frame is None else self._frame.copy()
        if frame is None:
            self.get_logger().warn('AprilTag: no camera frame yet')
            return
        ts = self._stamp()
        pose = self._pose_str()
        result = run_apriltag(frame)
        vis = frame.copy()
        lines = [f'[APRILTAG] {ts} robot_pose {pose}']

        if result['ok']:
            for h in result['hits']:
                pts = np.array(h['corners'], dtype=np.int32)
                cv2.polylines(vis, [pts], True, (255, 128, 0), 2)
                cv2.putText(
                    vis, f"id={h['id']}", (int(h['cx']), int(h['cy']) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 128, 0), 2)
                lines.append(
                    f"  id={h['id']} "
                    f"img_xy=({h['cx']:.1f},{h['cy']:.1f}) "
                    f"robot_pose=({self._pose[0]:.3f},{self._pose[1]:.3f},"
                    f"{self._pose[2]:.3f})")
        else:
            lines.append(f"  {result['message']}")

        cv2.putText(
            vis, f'AprilTag  {pose}  {ts}', (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

        img_path = self._out / f'apriltag_{ts}.jpg'
        cv2.imwrite(str(img_path), vis)
        for line in lines:
            self._append_txt(line)
        self._append_txt(f'  image={img_path.name}')
        self.get_logger().info(
            f'AprilTag saved {img_path.name} — {result["message"]}')
        self._show('AprilTag capture', vis)


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description='Teleop YOLO + AprilTag capture')
    parser.add_argument(
        '--out', default=str(Path(__file__).resolve().parent / 'captures'),
        help='folder for images + detections.txt')
    parser.add_argument('--yolo-btn', type=int, default=0, help='Joy button for YOLO (A=0)')
    parser.add_argument('--tag-btn', type=int, default=1, help='Joy button for AprilTag (B=1)')
    parser.add_argument('--conf', type=float, default=0.4, help='YOLO confidence')
    parser.add_argument(
        '--no-live', action='store_true',
        help='disable continuous live preview window')
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = TeleopCaptureNode(
        out_dir=Path(args.out).expanduser(),
        yolo_btn=args.yolo_btn,
        tag_btn=args.tag_btn,
        conf=args.conf,
        live_view=not args.no_live,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
