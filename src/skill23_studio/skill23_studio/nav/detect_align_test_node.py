#!/usr/bin/env python3
"""Bench trigger for Detect+Align without Nav2 (second terminal).

Terminal 1:  ros2 launch skill23_studio test_detect_align.launch.py
             (OMS rotation homes, vertical homes DOWN)
Terminal 2:  ros2 run skill23_studio studio_detect_align_test

  1. vertical servo UP (camera can see gripper)
  2. send /studio/detect_align/cmd  (worker enables camera, snapshot YOLO,
     OMS rotate, verify, depth)
  3. print status / OMS angle / bbox / depth
  4. streams off
"""

from __future__ import annotations

import json
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import SetBool


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
        self.declare_parameter('cmd_topic', '/studio/detect_align/cmd')
        self.declare_parameter('result_topic', '/studio/detect_align/result')
        self.declare_parameter('status_topic', '/studio/detect_align/status')
        self.declare_parameter('need_frames', 3)
        self.declare_parameter('stream_wait_sec', 12.0)
        self.declare_parameter('result_wait_sec', 90.0)
        self.declare_parameter('timeout_sec', 0.0)

        self._need_frames = max(1, int(self.get_parameter('need_frames').value))
        self._stream_wait = max(2.0, float(self.get_parameter('stream_wait_sec').value))
        self._result_wait = max(10.0, float(self.get_parameter('result_wait_sec').value))
        try:
            raw_to = float(self.get_parameter('timeout_sec').value or 0.0)
        except (TypeError, ValueError):
            raw_to = 0.0
        self._cmd_timeout = raw_to if raw_to > 0.0 else 0.0

        self._color_n = 0
        self._color_last: Optional[Image] = None
        self._color_t0 = 0.0
        self._depth_n = 0
        self._depth_last: Optional[Image] = None
        self._status: dict = {}
        self._status_key = ''
        self._result: Optional[dict] = None
        self._result_evt = threading.Event()
        self._depth_on = False
        self._want_depth = False
        self._done = False

        self._cmd_pub = self.create_publisher(
            String, str(self.get_parameter('cmd_topic').value), 10)
        self._vert_pub = self.create_publisher(String, '/oms/vertical/cmd', 10)
        self._vert_result: Optional[dict] = None
        self._vert_evt = threading.Event()
        self._toggle_color = self.create_client(SetBool, '/camera/toggle_color')
        self._toggle_depth = self.create_client(SetBool, '/camera/toggle_depth')
        self.create_subscription(
            Image, str(self.get_parameter('color_topic').value),
            self._on_color, qos_profile_sensor_data)
        self.create_subscription(
            Image, str(self.get_parameter('depth_topic').value),
            self._on_depth, qos_profile_sensor_data)
        self.create_subscription(
            String, str(self.get_parameter('status_topic').value),
            self._on_status, 10)
        self.create_subscription(
            String, str(self.get_parameter('result_topic').value),
            self._on_result, 10)
        self.create_subscription(
            String, '/oms/vertical/result', self._on_vert_result, 10)
        self.create_subscription(
            String, '/oms/rotation/status', self._on_oms_status, 10)

        self.get_logger().info(
            '[detect_align_test] ready — vertical UP, then Detect+Align '
            '(snapshot YOLO + OMS rotate + depth). No Nav2.')
        threading.Thread(target=self._run, daemon=True).start()

    def _on_color(self, msg: Image) -> None:
        if self._color_n == 0:
            self._color_t0 = time.time()
        self._color_n += 1
        self._color_last = msg

    def _on_depth(self, msg: Image) -> None:
        self._depth_n += 1
        self._depth_last = msg

    def _on_status(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if not isinstance(data, dict):
            return
        self._status = data
        key = (
            f'{data.get("state")}|{data.get("message")}|'
            f'{data.get("confirm")}|{data.get("depth_m")}'
        )
        if key == self._status_key:
            return
        self._status_key = key
        state = str(data.get('state') or '')
        u = data.get('u_norm')
        u_s = f' u={float(u):+.3f}' if isinstance(u, (int, float)) else ''
        oms = data.get('oms_angle_deg')
        oms_s = f' oms={float(oms):+.1f}°' if isinstance(oms, (int, float)) else ''
        grip = data.get('gripper_u')
        grip_s = f' grip_u={float(grip):+.3f}' if isinstance(grip, (int, float)) else ''
        conf = data.get('conf')
        conf_s = f' conf={float(conf):.2f}' if isinstance(conf, (int, float)) else ''
        name = str(data.get('class_name') or '')
        name_s = f' {name}' if name else ''
        confirm = data.get('confirm')
        need = data.get('confirm_need')
        cf = ''
        if confirm is not None and need:
            cf = f' confirm={confirm}/{need}'
        depth = data.get('depth_m')
        d_s = f' depth={float(depth):.3f}m' if isinstance(depth, (int, float)) else ''
        self.get_logger().info(
            f'[detect_align_test] STATUS {state}{name_s}{conf_s}{u_s}{oms_s}{grip_s}{cf}{d_s}'
            f'  {data.get("message") or ""}'.rstrip())
        if state in ('aligned', 'wait_depth', 'read_depth'):
            self._want_depth = True

    def _on_result(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            self.get_logger().warn(f'[detect_align_test] bad result JSON: {msg.data}')
            return
        self._result = data if isinstance(data, dict) else None
        self._result_evt.set()

    def _on_vert_result(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if isinstance(data, dict):
            self._vert_result = data
            cid = str(data.get('cmd_id') or '')
            if cid.startswith('test_vert') or str(data.get('pose') or '') == 'up':
                self._vert_evt.set()
            self.get_logger().info(
                f'[detect_align_test] VERTICAL ok={bool(data.get("success"))} '
                f'{data.get("message") or ""} pose={data.get("pose")}')

    def _on_oms_status(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if not isinstance(data, dict):
            return
        key = f'{data.get("state")}|{data.get("homed")}'
        if getattr(self, '_oms_key', '') == key:
            return
        self._oms_key = key
        ang = data.get('angle_deg')
        ang_s = f'{float(ang):+.2f}°' if isinstance(ang, (int, float)) else '?'
        self.get_logger().info(
            f'[detect_align_test] OMS rotate state={data.get("state")} '
            f'homed={bool(data.get("homed"))} angle={ang_s}')

    def _wait_service(self, client, name: str, timeout: float) -> bool:
        t0 = time.time()
        last = 0.0
        while rclpy.ok() and (time.time() - t0) < timeout:
            if client.service_is_ready():
                return True
            if time.time() - last >= 1.0:
                self.get_logger().warn(
                    f'[detect_align_test] waiting for {name} '
                    f'({timeout - (time.time() - t0):.0f}s left)')
                last = time.time()
            time.sleep(0.2)
        return False

    def _have_color(self) -> bool:
        return self._color_n >= self._need_frames and self._color_last is not None

    def _have_depth(self) -> bool:
        return self._depth_n >= 1 and self._depth_last is not None

    def _toggle(self, client, name: str, wanted: bool) -> bool:
        if not client.service_is_ready():
            self.get_logger().warn(f'[detect_align_test] {name} not ready')
            return False
        req = SetBool.Request()
        req.data = bool(wanted)
        fut = client.call_async(req)
        t0 = time.time()
        while rclpy.ok() and not fut.done() and (time.time() - t0) < 3.0:
            time.sleep(0.05)
        if not fut.done():
            self.get_logger().warn(
                f'[detect_align_test] {name} reply timed out '
                '(Already ON/OFF is OK — judging by frames)')
            return False
        try:
            res = fut.result()
        except Exception as extra:
            self.get_logger().warn(f'[detect_align_test] {name} failed: {extra}')
            return False
        ok = _ok_toggle(res, wanted)
        text = str(getattr(res, 'message', '') or '')
        self.get_logger().info(
            f'[detect_align_test] {name}={"ON" if wanted else "OFF"} '
            f'{"ok" if ok else "FAIL"} {text}'.strip())
        return ok

    def _ensure_color(self) -> bool:
        if self._have_color():
            self.get_logger().info(
                '[detect_align_test] color already streaming — skip toggle ON')
            return True
        if self._toggle_color.service_is_ready():
            self._toggle(self._toggle_color, 'toggle_color', True)
        return self._wait_color()

    def _enable_depth(self) -> None:
        if self._depth_on:
            return
        self._depth_on = True
        if self._have_depth():
            self.get_logger().info(
                '[detect_align_test] depth already streaming — skip toggle ON')
            return
        self.get_logger().info(
            '[detect_align_test] === enable depth stream (object aligned) ===')
        self._toggle(self._toggle_depth, 'toggle_depth', True)

    def _wait_color(self) -> bool:
        t0 = time.time()
        last = 0.0
        while rclpy.ok() and (time.time() - t0) < self._stream_wait:
            if self._have_color():
                msg = self._color_last
                dt = max(1e-3, time.time() - (self._color_t0 or t0))
                fps = (self._color_n - 1) / dt if self._color_n > 1 else 0.0
                self.get_logger().info(
                    f'[detect_align_test] color OK  {msg.width}x{msg.height} '
                    f'{msg.encoding or "?"}  {self._color_n} frames  '
                    f'~{fps:.1f} fps')
                return True
            if time.time() - last >= 1.0:
                self.get_logger().info(
                    f'[detect_align_test] waiting for color frames '
                    f'({self._color_n}/{self._need_frames})')
                last = time.time()
            time.sleep(0.1)
        self.get_logger().error(
            f'[detect_align_test] no color stream after {self._stream_wait:.0f}s '
            f'({self._color_n} frames)')
        return False

    def _wait_cmd_sub(self) -> bool:
        t0 = time.time()
        last = 0.0
        while rclpy.ok() and (time.time() - t0) < 15.0:
            nsub = int(self._cmd_pub.get_subscription_count())
            if nsub >= 1:
                return True
            if time.time() - last >= 1.0:
                self.get_logger().warn(
                    '[detect_align_test] waiting for detect_align on '
                    '/studio/detect_align/cmd')
                last = time.time()
            time.sleep(0.2)
        self.get_logger().error(
            '[detect_align_test] detect_align node not listening — '
            'is test_detect_align.launch.py running?')
        return False

    def _run(self) -> None:
        try:
            self._run_inner()
        except Exception as extra:
            self.get_logger().error(f'[detect_align_test] crashed: {extra}')
        self._done = True

    def _run_inner(self) -> None:
        self.get_logger().info(
            '[detect_align_test] === 1/6 wait for camera (toggle or frames) ===')
        t0 = time.time()
        last = 0.0
        while rclpy.ok() and (time.time() - t0) < 20.0:
            if self._toggle_color.service_is_ready() or self._color_n >= 1:
                break
            if time.time() - last >= 1.0:
                self.get_logger().warn(
                    '[detect_align_test] waiting for /camera/toggle_color '
                    f'or color frames ({20.0 - (time.time() - t0):.0f}s left)')
                last = time.time()
            time.sleep(0.2)
        if not self._toggle_color.service_is_ready() and self._color_n < 1:
            self.get_logger().error(
                '[detect_align_test] FAIL — camera not visible. '
                'Run THIS node the same way as terminal 1 (both sudo -E, '
                'or both without sudo). Leave launch running.')
            return

        self.get_logger().info(
            '[detect_align_test] === 2/6 color stream ===')
        if not self._ensure_color():
            return

        self.get_logger().info(
            '[detect_align_test] === 3/6 vertical UP (gripper in camera view) ===')
        self._vert_result = None
        self._vert_evt.clear()
        v_id = f'test_vert_{int(time.time())}'
        self._vert_pub.publish(String(data=json.dumps({
            'cmd_id': v_id, 'mode': 'up'})))
        t_v = time.time()
        while rclpy.ok() and not self._vert_evt.is_set() and (time.time() - t_v) < 8.0:
            time.sleep(0.05)
        if not self._vert_evt.is_set():
            self.get_logger().warn(
                '[detect_align_test] no vertical result — detect_align will retry up')

        self.get_logger().info(
            '[detect_align_test] === 4/6 send Detect+Align (snapshot + OMS) ===')
        if not self._wait_cmd_sub():
            self._toggle(self._toggle_color, 'toggle_color', False)
            return
        cmd_id = f'test_detect_align_{int(time.time())}'
        payload = {'cmd_id': cmd_id, 'mode': 'detect_align'}
        if self._cmd_timeout > 0.0:
            payload['timeout_sec'] = self._cmd_timeout
        self._result = None
        self._result_evt.clear()
        self._cmd_pub.publish(String(data=json.dumps(payload)))
        self.get_logger().info(
            f'[detect_align_test] SEND id={cmd_id}  '
            'watching /studio/detect_align/status')

        self.get_logger().info(
            '[detect_align_test] === 5/6 wait aligned + depth ===')
        t_res = time.time()
        while rclpy.ok() and not self._result_evt.is_set():
            if self._want_depth and not self._depth_on:
                self._enable_depth()
            if time.time() - t_res >= self._result_wait:
                self.get_logger().error(
                    f'[detect_align_test] FAIL — no result after {self._result_wait:.0f}s')
                self._shutdown_streams()
                return
            time.sleep(0.1)

        data = self._result or {}
        ok = bool(data.get('success'))
        optical = data.get('depth_m')
        horiz = data.get('depth_horizontal_m')
        tilt = data.get('camera_tilt_deg')
        name = data.get('class_name') or ''
        u = data.get('u_norm')
        bbox = data.get('bbox')
        self.get_logger().info(
            f'[detect_align_test] RESULT ok={ok}  {data.get("message") or ""}')
        if isinstance(optical, (int, float)):
            extra = ''
            if isinstance(horiz, (int, float)):
                extra += f'  horizontal={float(horiz):.3f} m'
            if isinstance(tilt, (int, float)):
                extra += f'  tilt={float(tilt):.1f} deg'
            self.get_logger().info(
                f'[detect_align_test] DEPTH  {name or "object"}  '
                f'optical={float(optical):.3f} m{extra}')
        else:
            self.get_logger().warn(
                '[detect_align_test] DEPTH  (none) — aligned but no valid sample')
        if u is not None:
            self.get_logger().info(
                f'[detect_align_test] u_norm={u}  bbox={bbox}  '
                f'oms={data.get("oms_angle_deg")}  gripper_u={data.get("gripper_u")}  '
                f'depth_frames_seen={self._depth_n}')

        self.get_logger().info(
            '[detect_align_test] === 6/6 disable color + depth streams ===')
        self._shutdown_streams()
        if ok and isinstance(optical, (int, float)):
            self.get_logger().info('[detect_align_test] DONE — pass')
        elif ok:
            self.get_logger().warn('[detect_align_test] DONE — aligned, no depth')
        else:
            self.get_logger().error('[detect_align_test] DONE — fail')

    def _shutdown_streams(self) -> None:
        self._toggle(self._toggle_depth, 'toggle_depth', False)
        self._toggle(self._toggle_color, 'toggle_color', False)
        self.get_logger().info('[detect_align_test] streams OFF')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectAlignTestNode()
    try:
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.1)
        # Let the last logs flush.
        t_end = time.time() + 0.6
        while rclpy.ok() and time.time() < t_end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        try:
            node._shutdown_streams()
        except Exception:
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
