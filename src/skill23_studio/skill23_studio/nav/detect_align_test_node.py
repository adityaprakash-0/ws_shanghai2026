#!/usr/bin/env python3
"""Lightweight Detect+Align vision bench — no OMS rotation.

One snapshot per START: color ON → YOLO → depth → debug image → streams OFF.
Press START again to repeat.

Terminal 1:  ros2 launch skill23_studio test_detect_align.launch.py robot:=allterrain
Terminal 2:  ros2 run skill23_studio studio_detect_align_test
START:       ros2 topic pub --once /studio/detect_align_test/start std_msgs/msg/Empty {}
View:        rqt_image_view /studio/detect_align/debug_image
"""

from __future__ import annotations

import json
import math
import threading
import time
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Empty, String
from std_srvs.srv import SetBool

from skill23_studio.detect_align_paths import yolov8n_ncnn_dir
from skill23_studio.nav.detect_align_geometry import (
    CameraInfoCache,
    TargetGeometry,
    bbox_center_px,
    bearing_deg_from_pixel,
    compute_target_geometry,
    pitch_rad_from_param,
    u_norm_from_pixel,
    yaw_direction,
)
from skill23_studio.nav.detect_align_node import (
    _depth_to_meters,
    _image_to_rgb,
    _rgb_to_bgr_msg,
    _sample_depth,
)


def _ok_toggle(res, wanted: bool) -> bool:
    if bool(getattr(res, 'success', False)):
        return True
    msg = str(getattr(res, 'message', '') or '').lower()
    if wanted and 'already on' in msg:
        return True
    if (not wanted) and 'already off' in msg:
        return True
    return False


class DetectAlignTestNode(Node):
    def __init__(self) -> None:
        super().__init__('detect_align_test')
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('color_camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('depth_camera_info_topic', '/camera/depth/camera_info')
        self.declare_parameter('debug_image_topic', '/studio/detect_align/debug_image')
        self.declare_parameter('start_topic', '/studio/detect_align_test/start')
        self.declare_parameter('model_dir', '')
        self.declare_parameter('input_size', 320)
        self.declare_parameter('conf_threshold', 0.35)
        self.declare_parameter('nms_iou', 0.45)
        self.declare_parameter('num_threads', 1)
        self.declare_parameter('target_class', -1)
        self.declare_parameter('target_name', '')
        self.declare_parameter('min_box_area_frac', 0.002)
        self.declare_parameter('hfov_deg', 69.0)
        self.declare_parameter('angular_sign', 1.0)
        self.declare_parameter('center_u_offset', 0.0)
        self.declare_parameter('camera_tilt_deg', 35.0)
        self.declare_parameter('camera_tilt_from_vertical', False)
        self.declare_parameter('depth_samples', 3)
        self.declare_parameter('depth_roi_frac', 0.08)
        self.declare_parameter('depth_timeout_sec', 6.0)
        self.declare_parameter('need_frames', 4)
        self.declare_parameter('skip_frames', 3)
        self.declare_parameter('stream_wait_sec', 12.0)
        self.declare_parameter('oms_min_deg', -90.0)
        self.declare_parameter('oms_max_deg', 90.0)

        raw_model = str(self.get_parameter('model_dir').value or '').strip()
        self._model_dir = raw_model or yolov8n_ncnn_dir()
        self._target_class = int(self.get_parameter('target_class').value)
        self._target_name = str(self.get_parameter('target_name').value or '').strip()
        self._min_area_frac = float(self.get_parameter('min_box_area_frac').value)
        self._hfov = float(self.get_parameter('hfov_deg').value)
        asig = float(self.get_parameter('angular_sign').value)
        self._asign = 1.0 if asig >= 0.0 else -1.0
        self._u_off = float(self.get_parameter('center_u_offset').value)
        self._tilt = pitch_rad_from_param(
            float(self.get_parameter('camera_tilt_deg').value),
            bool(self.get_parameter('camera_tilt_from_vertical').value))
        self._depth_samples = max(1, int(self.get_parameter('depth_samples').value))
        self._depth_roi = float(self.get_parameter('depth_roi_frac').value)
        self._depth_timeout = max(1.0, float(self.get_parameter('depth_timeout_sec').value))
        self._need_frames = max(1, int(self.get_parameter('need_frames').value))
        self._skip_frames = max(0, int(self.get_parameter('skip_frames').value))
        self._stream_wait = max(2.0, float(self.get_parameter('stream_wait_sec').value))
        self._oms_min = float(self.get_parameter('oms_min_deg').value)
        self._oms_max = float(self.get_parameter('oms_max_deg').value)
        self._debug_topic = str(self.get_parameter('debug_image_topic').value)
        self._cam_info = CameraInfoCache(self._hfov)

        self._color_n = 0
        self._color: Optional[Image] = None
        self._depth: Optional[Image] = None
        self._depth_stamp = 0.0
        self._detector = None
        self._busy = False
        self._start_evt = threading.Event()

        self._debug_pub = self.create_publisher(Image, self._debug_topic, 1)
        self._toggle_color = self.create_client(SetBool, '/camera/toggle_color')
        self._toggle_depth = self.create_client(SetBool, '/camera/toggle_depth')
        color_topic = str(self.get_parameter('color_topic').value)
        depth_topic = str(self.get_parameter('depth_topic').value)
        self.create_subscription(
            Image, color_topic, self._on_color, qos_profile_sensor_data)
        self.create_subscription(
            Image, depth_topic, self._on_depth, qos_profile_sensor_data)
        color_info_topic = str(self.get_parameter('color_camera_info_topic').value)
        depth_info_topic = str(self.get_parameter('depth_camera_info_topic').value)
        if color_info_topic:
            self.create_subscription(
                CameraInfo, color_info_topic, self._on_color_info, qos_profile_sensor_data)
        if depth_info_topic:
            self.create_subscription(
                CameraInfo, depth_info_topic, self._on_depth_info, qos_profile_sensor_data)
        self.create_subscription(
            Empty, str(self.get_parameter('start_topic').value),
            self._on_start, 10)

        self._log(
            'ready — vision-only (no OMS). '
            f'target={self._target_name or "any"} class={self._target_class}  '
            f'conf>={float(self.get_parameter("conf_threshold").value):.2f}  '
            f'tilt={math.degrees(self._tilt):.0f}°')
        self._log('Type start + Enter here, or from another terminal:')
        self._log(
            '  ros2 topic pub --once /studio/detect_align_test/start '
            'std_msgs/msg/Empty {}')
        self._log(f'VIEW:  rqt_image_view {self._debug_topic}')
        self._log('waiting for START…')
        threading.Thread(target=self._stdin_loop, daemon=True).start()

    def _request_start(self, source: str = 'topic') -> None:
        if self._busy:
            self._log(
                f'busy — ignore START from {source} (wait for current run)',
                'warn')
            return
        self._log(f'START received ({source})')
        self._start_evt.set()

    def _stdin_loop(self) -> None:
        import sys
        while rclpy.ok():
            try:
                line = sys.stdin.readline()
            except Exception:
                break
            if not line:
                time.sleep(0.2)
                continue
            cmd = line.strip().lower()
            if cmd in ('start', 'go', 'run', 's'):
                self._request_start('keyboard')
            elif cmd in ('q', 'quit', 'exit'):
                self._log('quit requested')
                rclpy.shutdown()
                break
            elif cmd:
                self._log(
                    f'unknown command {line.strip()!r} — type start, or q to quit',
                    'warn')

    def _log(self, msg: str, level: str = 'info') -> None:
        text = f'[detect_align_test] {msg}'
        if level == 'error':
            self.get_logger().error(text)
        elif level == 'warn':
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)

    def _on_start(self, _msg: Empty) -> None:
        self._request_start('topic')

    def _on_color(self, msg: Image) -> None:
        self._color_n += 1
        self._color = msg

    def _on_depth(self, msg: Image) -> None:
        self._depth = msg
        self._depth_stamp = time.time()

    def _on_color_info(self, msg: CameraInfo) -> None:
        self._cam_info.set_color_info(msg)

    def _on_depth_info(self, msg: CameraInfo) -> None:
        self._cam_info.set_depth_info(msg)

    def _toggle(self, client, label: str, wanted: bool) -> bool:
        if client is None or not client.service_is_ready():
            self._log(f'{label} service not ready', 'warn')
            return False
        req = SetBool.Request()
        req.data = bool(wanted)
        fut = client.call_async(req)
        t0 = time.time()
        while rclpy.ok() and not fut.done() and (time.time() - t0) < 3.0:
            time.sleep(0.05)
        if not fut.done():
            self._log(f'{label} toggle timeout', 'error')
            return False
        try:
            res = fut.result()
        except Exception as exc:
            self._log(f'{label} toggle failed: {exc}', 'error')
            return False
        ok = _ok_toggle(res, wanted)
        note = str(getattr(res, 'message', '') or '')
        self._log(f'{label} {"ON" if wanted else "OFF"} {"ok" if ok else "FAIL"} {note}'.strip())
        return ok

    def _streams_off(self) -> None:
        self._toggle(self._toggle_color, 'color', False)
        self._toggle(self._toggle_depth, 'depth', False)

    def _wait_frames(self, need: int, timeout: float) -> bool:
        start = self._color_n
        t0 = time.time()
        last_log = 0.0
        while rclpy.ok() and (time.time() - t0) < timeout:
            got = self._color_n - start
            if got >= need and self._color is not None:
                msg = self._color
                self._log(
                    f'color OK  {msg.width}x{msg.height}  '
                    f'{msg.encoding or "?"}  {got} new frames')
                return True
            if time.time() - last_log >= 1.0:
                self._log(f'waiting color ({got}/{need})')
                last_log = time.time()
            time.sleep(0.05)
        self._log(f'no color after {timeout:.0f}s', 'error')
        return False

    def _grab_snapshot(self) -> Optional[Image]:
        need = self._color_n + self._skip_frames
        deadline = time.time() + self._stream_wait
        while rclpy.ok() and time.time() < deadline:
            if self._color is not None and self._color_n >= need:
                return self._color
            time.sleep(0.05)
        return None

    def _ensure_detector(self):
        if self._detector is not None:
            return self._detector
        from skill23_studio.nav.yolov8_ncnn import YoloV8Ncnn
        self._log(f'loading NCNN model from {self._model_dir}')
        self._detector = YoloV8Ncnn(
            self._model_dir,
            input_size=int(self.get_parameter('input_size').value),
            conf_threshold=float(self.get_parameter('conf_threshold').value),
            nms_iou=float(self.get_parameter('nms_iou').value),
            num_threads=int(self.get_parameter('num_threads').value),
        )
        return self._detector

    def _drop_detector(self) -> None:
        det = self._detector
        self._detector = None
        if det is not None:
            try:
                det.close()
            except Exception:
                pass

    def _detect(self, snap: Image):
        rgb = _image_to_rgb(snap)
        if rgb is None:
            return None, None, None, f'bad encoding {snap.encoding}'
        det = self._ensure_detector()
        dets = det.detect(rgb)
        h, w = rgb.shape[:2]
        min_area = self._min_area_frac * float(w * h)
        obj = det.pick(
            dets,
            class_id=self._target_class,
            class_name=self._target_name,
            min_area=min_area,
        )
        if obj is None:
            note = 'no target detected'
            if dets:
                top = sorted(dets, key=lambda d: d.conf, reverse=True)[:5]
                parts = [f'{d.name}({float(d.conf):.2f})' for d in top]
                filt = self._target_name or (
                    f'class={self._target_class}' if self._target_class >= 0 else 'any')
                note = (
                    f'no target after filter ({filt}); '
                    f'YOLO saw {len(dets)}: {", ".join(parts)}')
            else:
                note = (
                    f'YOLO zero boxes above conf '
                    f'{float(self.get_parameter("conf_threshold").value):.2f}')
            return None, (w, h), rgb, note
        return obj, (w, h), rgb, ''

    def _read_depth_z(self, u_norm: float, v_norm: float) -> Optional[float]:
        samples: List[float] = []
        deadline = time.time() + self._depth_timeout
        while rclpy.ok() and time.time() < deadline:
            msg = self._depth
            if msg is None or time.time() - self._depth_stamp > 1.0:
                time.sleep(0.05)
                continue
            depth = _depth_to_meters(msg)
            if depth is None:
                time.sleep(0.05)
                continue
            val = _sample_depth(depth, u_norm, v_norm, self._depth_roi)
            if val is None:
                time.sleep(0.05)
                continue
            samples.append(val)
            self._log(f'depth sample {len(samples)}/{self._depth_samples} Z={val:.3f} m')
            if len(samples) >= self._depth_samples:
                break
            time.sleep(0.05)
        if not samples:
            return None
        return float(np.median(np.array(samples, dtype=np.float32)))

    def _publish_debug(
            self, rgb: np.ndarray, snap: Image, obj, u_err: float,
            bearing: float, geom: Optional[TargetGeometry],
            color_intr) -> None:
        direction = yaw_direction(bearing)
        img = rgb.copy()
        h, w = img.shape[:2]
        cx_line = w // 2
        cv2.line(img, (cx_line, 0), (cx_line, h - 1), (0, 255, 255), 1)
        if obj is not None:
            x1, y1, x2, y2 = int(obj.x1), int(obj.y1), int(obj.x2), int(obj.y2)
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            oc = int(0.5 * (obj.x1 + obj.x2))
            oc_y = int(0.5 * (obj.y1 + obj.y2))
            cv2.circle(img, (oc, oc_y), 4, (255, 0, 0), -1)
            cv2.line(img, (cx_line, oc_y), (oc, oc_y), (255, 255, 0), 1)
        lines = [
            f'{obj.name if obj else "?"} conf={float(obj.conf):.2f}' if obj else 'no object',
            f'u_err={u_err:+.3f}  yaw={bearing:+.1f} deg {direction}',
            f'turn camera to center line on object  fx={color_intr.fx:.1f}',
        ]
        if geom is not None:
            lat_side = 'center'
            if geom.lateral_m > 0.005:
                lat_side = 'L'
            elif geom.lateral_m < -0.005:
                lat_side = 'R'
            lines.append(
                f'Z={geom.z_depth_m:.3f}m  fwd={geom.forward_m:.3f}m  '
                f'lat={geom.lateral_m:+.3f}m ({lat_side})')
            lines.append(
                f'floor={geom.floor_dist_m:.3f}m  (linear rail use fwd after turn)')
        else:
            lines.append('depth=n/a')
        for i, text in enumerate(lines):
            cv2.putText(img, text, (8, 22 + i * 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(img, text, (8, 22 + i * 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 128, 255), 1, cv2.LINE_AA)
        try:
            self._debug_pub.publish(_rgb_to_bgr_msg(img, snap.header.stamp))
        except Exception as exc:
            self._log(f'debug image publish failed: {exc}', 'warn')

    def _run_once(self) -> None:
        self._busy = True
        try:
            self._log('=== START run ===')
            self._color_n = 0
            self._depth = None

            self._log('1/5 enable color stream')
            if not self._toggle(self._toggle_color, 'color', True):
                return
            if not self._wait_frames(self._need_frames, self._stream_wait):
                return

            self._log('2/5 grab snapshot + YOLO')
            snap = self._grab_snapshot()
            self._toggle(self._toggle_color, 'color', False)
            if snap is None:
                self._log('snapshot timeout', 'error')
                return

            obj, size, rgb, note = self._detect(snap)
            if obj is None or rgb is None:
                self._log(note or 'detection failed', 'warn')
                blank = rgb if rgb is not None else np.zeros((360, 640, 3), dtype=np.uint8)
                intr = self._cam_info.color_or_fallback(blank.shape[1], blank.shape[0])
                self._publish_debug(
                    blank, snap, None, 0.0, 0.0, None, intr)
                return

            w, h = size
            u_px, v_px = bbox_center_px(obj)
            color_intr = self._cam_info.color_or_fallback(w, h)
            u_err = u_norm_from_pixel(u_px, w, self._u_off)
            bearing = bearing_deg_from_pixel(
                u_px, color_intr, self._asign, self._oms_min, self._oms_max)
            direction = yaw_direction(bearing)
            bbox = [round(obj.x1, 1), round(obj.y1, 1), round(obj.x2, 1), round(obj.y2, 1)]
            self._log(
                f'DETECT  class={obj.name} id={obj.class_id} conf={float(obj.conf):.2f}  '
                f'bbox={bbox}')
            self._log(
                f'BEARING u_px={u_px:.1f} fx={color_intr.fx:.1f}  '
                f'→ {bearing:+.1f}° ({direction})  (+ACW / −CW)')

            self._log('3/5 enable depth stream')
            geom: Optional[TargetGeometry] = None
            if self._toggle(self._toggle_depth, 'depth', True):
                time.sleep(0.4)
                self._log('4/5 sample Z-depth + compute fwd from intrinsics/tilt')
                u_norm = u_px / max(1.0, float(w))
                v_norm = v_px / max(1.0, float(h))
                z_depth = self._read_depth_z(u_norm, v_norm)
                if z_depth is not None:
                    dw = int(self._depth.width) if self._depth is not None else w
                    dh = int(self._depth.height) if self._depth is not None else h
                    depth_intr = self._cam_info.depth_or_fallback(dw, dh)
                    geom = compute_target_geometry(
                        u_px, v_px, z_depth,
                        color_intr, depth_intr, self._tilt,
                        self._asign, self._oms_min, self._oms_max)
                    self._log(
                        f'3D  Z={z_depth:.3f}m  fwd={geom.forward_m:.3f}m  '
                        f'lat={geom.lateral_m:+.3f}m  floor={geom.floor_dist_m:.3f}m  '
                        f'bearing={bearing:+.1f}°')
                else:
                    self._log('no valid depth samples', 'warn')
            else:
                self._log('depth stream unavailable — skipping distance', 'warn')

            self._log('5/5 publish debug image + disable streams')
            self._publish_debug(
                rgb, snap, obj, u_err, bearing, geom, color_intr)
            self._streams_off()
            self._drop_detector()

            summary = {
                'class_name': obj.name,
                'class_id': int(obj.class_id),
                'conf': round(float(obj.conf), 3),
                'bbox': bbox,
                'u_norm': round(u_err, 4),
                'bearing_deg': round(bearing, 2),
                'yaw_to_center_deg': round(bearing, 2),
                'direction': direction,
                'depth_z_m': None if geom is None else round(geom.z_depth_m, 4),
                'depth_m': None if geom is None else round(geom.z_depth_m, 4),
                'depth_forward_m': None if geom is None else round(geom.forward_m, 4),
                'depth_horizontal_m': None if geom is None else round(geom.forward_m, 4),
                'lateral_m': None if geom is None else round(geom.lateral_m, 4),
                'floor_dist_m': None if geom is None else round(geom.floor_dist_m, 4),
                'camera_tilt_deg': round(math.degrees(self._tilt), 1),
                'fx': round(color_intr.fx, 2),
                'fy': round(color_intr.fy, 2),
            }
            self._log(f'DONE  {json.dumps(summary)}')
            self._log('streams OFF — type start + Enter to repeat')
        finally:
            self._streams_off()
            self._drop_detector()
            self._busy = False

    def spin_loop(self) -> None:
        while rclpy.ok():
            if self._start_evt.wait(timeout=0.1):
                self._start_evt.clear()
                self._run_once()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectAlignTestNode()
    worker = threading.Thread(target=node.spin_loop, daemon=True)
    worker.start()
    try:
        while rclpy.ok() and worker.is_alive():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
