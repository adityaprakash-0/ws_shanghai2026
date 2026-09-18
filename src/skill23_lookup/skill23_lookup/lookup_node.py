#!/usr/bin/env python3
"""Waypoint lookup node — OCR digits or ArUco IDs.

Cmd (JSON on /studio/lookup/cmd):
  {cmd_id, mode: "ocr"|"aruco", confirm_frames?, retries?, timeout_sec?}

Result (JSON on /studio/lookup/result):
  {cmd_id, success, mode, next_wp?, message, detections?}

Idle until a cmd arrives. Subscribes to color only while active.
Publishes /studio/camera/action_need true/false so the Studio bridge
can toggle Orbbec streams.
"""

from __future__ import annotations

import json
import re
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Empty, String


ARUCO_DICTS = (
    'DICT_4X4_50',
    'DICT_4X4_100',
    'DICT_5X5_50',
    'DICT_5X5_100',
    'DICT_6X6_50',
    'DICT_6X6_250',
    'DICT_7X7_50',
    'DICT_ARUCO_ORIGINAL',
)

_DIGIT_RE = re.compile(r'(\d{1,3})')


def _json_loads(raw: str) -> dict:
    data = json.loads(raw or '{}')
    return data if isinstance(data, dict) else {}


class LookupNode(Node):
    def __init__(self) -> None:
        super().__init__('studio_lookup')
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('confirm_frames', 3)
        self.declare_parameter('retries', 3)
        self.declare_parameter('timeout_sec', 25.0)
        self.declare_parameter('ocr_allowlist', '0123456789')
        self.declare_parameter('warmup_ocr', True)

        self._color_topic = str(self.get_parameter('color_topic').value)
        self._default_confirm = max(1, int(self.get_parameter('confirm_frames').value))
        self._default_retries = max(1, int(self.get_parameter('retries').value))
        self._default_timeout = max(5.0, float(self.get_parameter('timeout_sec').value))
        self._allowlist = str(self.get_parameter('ocr_allowlist').value) or '0123456789'

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_bgr: Optional[np.ndarray] = None
        self._latest_stamp = 0.0
        self._image_sub = None

        self._cmd_id = ''
        self._mode = ''
        self._active = False
        self._deadline = 0.0
        self._confirm_need = self._default_confirm
        self._retries_left = self._default_retries
        self._confirm_value: Optional[int] = None
        self._confirm_count = 0
        self._attempt = 0
        self._last_frame_t = 0.0
        self._frames_seen = 0

        self._reader = None
        self._aruco_detectors: List[Tuple[str, Any]] = []
        self._init_aruco()

        self._cam_need_pub = self.create_publisher(Bool, '/studio/camera/action_need', 10)
        self._status_pub = self.create_publisher(String, '/studio/lookup/status', 10)
        self._result_pub = self.create_publisher(String, '/studio/lookup/result', 10)
        self.create_subscription(String, '/studio/lookup/cmd', self._on_cmd, 10)
        self.create_subscription(Empty, '/studio/lookup/cancel', self._on_cancel, 10)
        self.create_timer(0.1, self._tick)
        self.create_timer(1.0, self._publish_idle_status)

        if bool(self.get_parameter('warmup_ocr').value):
            threading.Thread(target=self._warmup_ocr, daemon=True, name='lookup_ocr_warm').start()

        self.get_logger().info(
            f'Lookup IDLE — color={self._color_topic} '
            f'aruco_dicts={len(self._aruco_detectors)} '
            f'confirm={self._default_confirm} retries={self._default_retries}')

    def _warmup_ocr(self) -> None:
        try:
            from skill23_lookup.prefetch_models import ensure_easyocr_models
            ensure_easyocr_models(force_download=False)
            self._ensure_reader()
            self.get_logger().info('EasyOCR warm-load OK (offline models ready)')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'EasyOCR warm-load failed: {exc}')

    def _init_aruco(self) -> None:
        if not hasattr(cv2, 'aruco'):
            self.get_logger().warn('cv2.aruco missing — lookup_aruco will fail')
            return
        params = cv2.aruco.DetectorParameters()
        for name in ARUCO_DICTS:
            dict_id = getattr(cv2.aruco, name, None)
            if dict_id is None:
                continue
            dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
            try:
                det = cv2.aruco.ArucoDetector(dictionary, params)
            except Exception:
                continue
            self._aruco_detectors.append((name, det))

    def _ensure_reader(self):
        if self._reader is not None:
            return self._reader
        import easyocr  # noqa: WPS433
        self._reader = easyocr.Reader(['en'], gpu=False, verbose=False)
        return self._reader

    def _set_cam_need(self, on: bool) -> None:
        self._cam_need_pub.publish(Bool(data=bool(on)))

    def _subscribe_color(self) -> None:
        if self._image_sub is not None:
            return
        self._image_sub = self.create_subscription(
            Image, self._color_topic, self._on_image, qos_profile_sensor_data)

    def _unsubscribe_color(self) -> None:
        if self._image_sub is None:
            return
        try:
            self.destroy_subscription(self._image_sub)
        except Exception:
            pass
        self._image_sub = None

    def _on_image(self, msg: Image) -> None:
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return
        with self._lock:
            self._latest_bgr = bgr
            self._latest_stamp = time.time()

    def _on_cancel(self, _msg: Empty) -> None:
        if not self._active:
            return
        self._finish(False, 'canceled', next_wp=None)

    def _on_cmd(self, msg: String) -> None:
        try:
            data = _json_loads(msg.data)
        except Exception:
            return
        mode = str(data.get('mode') or '').strip().lower()
        if mode in ('lookup_ocr', 'ocr', 'digit', 'number'):
            mode = 'ocr'
        elif mode in ('lookup_aruco', 'aruco', 'marker'):
            mode = 'aruco'
        else:
            self.get_logger().warn(f'lookup ignore unknown mode={mode!r}')
            return
        cmd_id = str(data.get('cmd_id') or f'lookup_{int(time.time() * 1000)}')
        try:
            confirm = int(data.get('confirm_frames', self._default_confirm) or self._default_confirm)
        except (TypeError, ValueError):
            confirm = self._default_confirm
        try:
            retries = int(data.get('retries', self._default_retries) or self._default_retries)
        except (TypeError, ValueError):
            retries = self._default_retries
        try:
            timeout = float(data.get('timeout_sec', self._default_timeout) or self._default_timeout)
        except (TypeError, ValueError):
            timeout = self._default_timeout
        confirm = max(1, min(20, confirm))
        retries = max(1, min(10, retries))
        timeout = max(5.0, min(120.0, timeout))

        if self._active:
            self._finish(False, 'preempted', next_wp=None)

        self._cmd_id = cmd_id
        self._mode = mode
        self._confirm_need = confirm
        self._retries_left = retries
        self._confirm_value = None
        self._confirm_count = 0
        self._attempt = 1
        self._frames_seen = 0
        self._last_frame_t = 0.0
        self._deadline = time.time() + timeout
        self._active = True
        with self._lock:
            self._latest_bgr = None
            self._latest_stamp = 0.0
        self._set_cam_need(True)
        self._subscribe_color()
        self.get_logger().info(
            f'lookup START id={cmd_id} mode={mode} confirm={confirm} '
            f'retries={retries} timeout={timeout:.0f}s')
        self._publish_status(f'running {mode}')

    def _get_frame(self) -> Optional[np.ndarray]:
        with self._lock:
            if self._latest_bgr is None:
                return None
            # Require a fresh frame after stream-on.
            if (time.time() - self._latest_stamp) > 1.5:
                return None
            return self._latest_bgr.copy()

    def _parse_ocr_number(self, frame: np.ndarray) -> Tuple[Optional[int], str]:
        reader = self._ensure_reader()
        # EasyOCR wants RGB.
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        try:
            results = reader.readtext(
                rgb,
                allowlist=self._allowlist,
                detail=1,
                paragraph=False,
            )
        except TypeError:
            results = reader.readtext(rgb, detail=1, paragraph=False)
        except Exception as exc:  # noqa: BLE001
            return None, f'ocr error: {exc}'

        numbers: List[Tuple[int, float]] = []
        for item in results or []:
            if not isinstance(item, (list, tuple)) or len(item) < 2:
                continue
            text = str(item[1] or '').strip()
            conf = float(item[2]) if len(item) > 2 else 0.0
            for m in _DIGIT_RE.finditer(text):
                try:
                    numbers.append((int(m.group(1)), conf))
                except ValueError:
                    continue
        if not numbers:
            return None, 'no digits'
        # Prefer highest confidence; ambiguous if two distinct high-conf values.
        numbers.sort(key=lambda t: t[1], reverse=True)
        best_val, best_conf = numbers[0]
        rivals = [v for v, c in numbers[1:] if v != best_val and c >= max(0.35, best_conf - 0.15)]
        if rivals:
            return None, f'ambiguous {best_val} vs {sorted(set(rivals))}'
        return best_val, f'ocr={best_val} conf={best_conf:.2f}'

    def _parse_aruco_id(self, frame: np.ndarray) -> Tuple[Optional[int], str]:
        if not self._aruco_detectors:
            return None, 'aruco unavailable'
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ids_found: List[int] = []
        for name, det in self._aruco_detectors:
            try:
                _corners, ids, _rej = det.detectMarkers(gray)
            except Exception:
                continue
            if ids is None:
                continue
            for row in ids:
                for v in np.asarray(row).reshape(-1):
                    ids_found.append(int(v))
            if ids_found:
                break
            del name
        if not ids_found:
            return None, 'no aruco'
        uniq = sorted(set(ids_found))
        if len(uniq) > 1:
            return None, f'ambiguous aruco {uniq}'
        return uniq[0], f'aruco={uniq[0]}'

    def _tick(self) -> None:
        if not self._active:
            return
        if time.time() > self._deadline:
            self._finish(False, f'{self._mode} timeout', next_wp=None)
            return
        frame = self._get_frame()
        if frame is None:
            self._publish_status(f'waiting_frames {self._mode}')
            return
        now = time.time()
        # Throttle detection ~4 Hz.
        if (now - self._last_frame_t) < 0.25:
            return
        self._last_frame_t = now
        self._frames_seen += 1

        if self._mode == 'ocr':
            value, detail = self._parse_ocr_number(frame)
        else:
            value, detail = self._parse_aruco_id(frame)

        if value is None:
            self._confirm_value = None
            self._confirm_count = 0
            self._publish_status(f'retry {detail} attempt={self._attempt}')
            # Consume a retry only when we have seen enough frames with nothing clear.
            if self._frames_seen >= (self._confirm_need * 2):
                self._retries_left -= 1
                self._frames_seen = 0
                self._attempt += 1
                if self._retries_left <= 0:
                    self._finish(False, f'{self._mode} failed: {detail}', next_wp=None)
            return

        if self._confirm_value == value:
            self._confirm_count += 1
        else:
            self._confirm_value = value
            self._confirm_count = 1
        self._publish_status(
            f'verify {value} {self._confirm_count}/{self._confirm_need} ({detail})')
        if self._confirm_count >= self._confirm_need:
            self._finish(True, f'{self._mode} ok {detail}', next_wp=int(value))

    def _finish(self, ok: bool, message: str, next_wp: Optional[int]) -> None:
        cmd_id = self._cmd_id
        mode = self._mode or 'lookup'
        self._active = False
        self._unsubscribe_color()
        self._set_cam_need(False)
        payload: Dict[str, Any] = {
            'cmd_id': cmd_id,
            'success': bool(ok),
            'mode': mode,
            'message': message,
            'frames': self._frames_seen,
        }
        if next_wp is not None:
            payload['next_wp'] = int(next_wp)
        self._result_pub.publish(String(data=json.dumps(payload)))
        self._publish_status('idle' if ok else f'failed {message}')
        self.get_logger().info(
            f'lookup DONE id={cmd_id} ok={ok} next_wp={next_wp} {message}')
        self._cmd_id = ''
        self._mode = ''

    def _publish_status(self, message: str) -> None:
        payload = {
            'state': 'running' if self._active else 'idle',
            'mode': self._mode or '',
            'message': message,
            'cmd_id': self._cmd_id,
            'confirm': self._confirm_count,
            'confirm_need': self._confirm_need,
            'next_wp': self._confirm_value,
        }
        self._status_pub.publish(String(data=json.dumps(payload)))

    def _publish_idle_status(self) -> None:
        if self._active:
            return
        self._publish_status('idle')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LookupNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._set_cam_need(False)
        except Exception:
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
