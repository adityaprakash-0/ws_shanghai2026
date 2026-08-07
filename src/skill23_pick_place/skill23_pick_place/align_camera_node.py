#!/usr/bin/env python3
"""Optional camera alignment using skill23_vision FindColor (centroid u)."""

from __future__ import annotations

import json
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

try:
    from skill23_vision.srv import FindColor, SetGate
    _HAS_VISION = True
except ImportError:
    FindColor = None  # type: ignore
    SetGate = None  # type: ignore
    _HAS_VISION = False


class AlignCameraNode(Node):
    def __init__(self):
        super().__init__('pp_align_camera')
        self.declare_parameter('cmd_vel_topic', '/pp/cmd_vel')
        self.declare_parameter('robot', 'mecanum')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('center_tol_px', 40)
        self.declare_parameter('confirm_frames', 10)
        self.declare_parameter('timeout_sec', 8.0)
        self.declare_parameter('kp', 0.002)
        self.declare_parameter('vmax', 0.12)
        self.declare_parameter('wmax', 0.3)

        self._robot = self.get_parameter('robot').value
        self._width = float(self.get_parameter('image_width').value)
        self._tol = float(self.get_parameter('center_tol_px').value)
        self._confirm_need = int(self.get_parameter('confirm_frames').value)
        self._timeout = float(self.get_parameter('timeout_sec').value)
        self._kp = float(self.get_parameter('kp').value)
        self._vmax = float(self.get_parameter('vmax').value)
        self._wmax = float(self.get_parameter('wmax').value)

        self._cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self._result_pub = self.create_publisher(String, '/pp/align/result', 10)
        self.create_subscription(String, '/pp/align/camera', self._on_cmd, 10)
        self._find_cli = None
        self._gate_cli = None
        if _HAS_VISION:
            self._find_cli = self.create_client(FindColor, '/adev/vision/find_color')
            self._gate_cli = self.create_client(SetGate, '/adev/vision/set_gate')
        self._lock = threading.Lock()
        self._busy = False
        self.get_logger().info(
            'Camera align ready' if _HAS_VISION else 'Camera align: skill23_vision not available')

    def _stop(self):
        self._cmd_pub.publish(Twist())

    def _result(self, cmd_id, ok, message, color=''):
        msg = String()
        msg.data = json.dumps({
            'cmd_id': cmd_id, 'success': ok, 'message': message,
            'mode': 'camera', 'color_name': color})
        self._result_pub.publish(msg)

    def _on_cmd(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except Exception as exc:
            self._result('', False, str(exc))
            return
        threading.Thread(target=self._run, args=(cmd,), daemon=True).start()

    def _call_find(self, colors, timeout_sec) -> dict:
        if self._find_cli is None:
            return {'found': False, 'message': 'vision unavailable'}
        if not self._find_cli.wait_for_service(timeout_sec=2.0):
            return {'found': False, 'message': 'find_color not ready'}
        req = FindColor.Request()
        req.colors = [str(c) for c in colors]
        req.timeout_sec = float(timeout_sec)
        fut = self._find_cli.call_async(req)
        deadline = time.time() + float(timeout_sec) + 2.0
        while rclpy.ok() and not fut.done() and time.time() < deadline:
            time.sleep(0.05)
        if not fut.done():
            return {'found': False, 'message': 'find timeout'}
        res = fut.result()
        return {
            'found': bool(res.found),
            'color_name': str(res.color_name or ''),
            'u': int(res.u),
            'v': int(res.v),
            'confidence': float(res.confidence),
            'message': str(res.message or ''),
        }

    def _set_gate(self, active: bool):
        if self._gate_cli is None:
            return
        if not self._gate_cli.wait_for_service(timeout_sec=1.0):
            return
        req = SetGate.Request()
        req.data = bool(active)
        fut = self._gate_cli.call_async(req)
        t0 = time.time()
        while rclpy.ok() and not fut.done() and time.time() - t0 < 2.0:
            time.sleep(0.05)

    def _run(self, cmd: dict):
        with self._lock:
            if self._busy:
                self._result(str(cmd.get('cmd_id', '')), False, 'busy')
                return
            self._busy = True
        cmd_id = str(cmd.get('cmd_id', ''))
        colors = cmd.get('colors') or ['red']
        timeout = float(cmd.get('timeout_sec', self._timeout))
        ok = False
        message = 'not found'
        color = ''
        confirm = 0
        t0 = time.time()
        try:
            self._set_gate(True)
            while rclpy.ok() and time.time() - t0 < timeout:
                # Short detection slices so we can servo between looks
                slice_t = min(1.0, timeout - (time.time() - t0))
                if slice_t <= 0:
                    break
                res = self._call_find(colors, slice_t)
                if not res.get('found'):
                    confirm = 0
                    # Search yaw
                    tw = Twist()
                    tw.angular.z = 0.15
                    self._cmd_pub.publish(tw)
                    time.sleep(0.2)
                    self._stop()
                    continue
                color = res.get('color_name', '')
                u = float(res.get('u', self._width / 2))
                err = u - (self._width / 2.0)
                if abs(err) <= self._tol:
                    confirm += 1
                    self._stop()
                    if confirm >= self._confirm_need:
                        ok, message = True, f'centered on {color}'
                        break
                else:
                    confirm = 0
                    corr = -self._kp * err
                    tw = Twist()
                    if self._robot in ('allterrain', 'grip'):
                        tw.angular.z = max(-self._wmax, min(self._wmax, corr * 2.0))
                    else:
                        tw.linear.y = max(-self._vmax, min(self._vmax, corr))
                        tw.angular.z = max(-self._wmax, min(self._wmax, corr * 0.5))
                    self._cmd_pub.publish(tw)
                    time.sleep(0.15)
                    self._stop()
            if not ok:
                message = res.get('message', message) if 'res' in dir() else message
        finally:
            self._stop()
            self._set_gate(False)
            with self._lock:
                self._busy = False
            self._result(cmd_id, ok, message, color)


def main(args=None):
    rclpy.init(args=args)
    node = AlignCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
