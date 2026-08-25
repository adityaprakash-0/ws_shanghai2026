#!/usr/bin/env python3
"""Wall/object alignment using front-left and front-right ultrasonic sensors.

Subscribes to two Range topics (front-left, front-right).  When a command
arrives on ``/custom_nav/align/us`` the node creeps and yaws until both
sensors read roughly the same distance (within tolerance), then stops.
"""

from __future__ import annotations

import json
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String


class AlignUsNode(Node):
    def __init__(self):
        super().__init__('cn_align_us')

        self.declare_parameter('us_left_topic', '/front_left_sonar/range')
        self.declare_parameter('us_right_topic', '/front_right_sonar/range')
        self.declare_parameter('tolerance_m', 0.015)
        self.declare_parameter('confirm_frames', 15)
        self.declare_parameter('timeout_sec', 8.0)
        self.declare_parameter('kp_yaw', 0.6)
        self.declare_parameter('wmax', 0.25)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        left_topic = str(self.get_parameter('us_left_topic').value)
        right_topic = str(self.get_parameter('us_right_topic').value)

        self._left: Optional[float] = None
        self._right: Optional[float] = None
        self.create_subscription(Range, left_topic, self._on_left, 10)
        self.create_subscription(Range, right_topic, self._on_right, 10)

        self._tol = float(self.get_parameter('tolerance_m').value)
        self._confirm_need = int(self.get_parameter('confirm_frames').value)
        self._timeout = float(self.get_parameter('timeout_sec').value)
        self._kp = float(self.get_parameter('kp_yaw').value)
        self._wmax = float(self.get_parameter('wmax').value)

        self._cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self._result_pub = self.create_publisher(
            String, '/custom_nav/align/result', 10)
        self.create_subscription(
            String, '/custom_nav/align/us', self._on_cmd, 10)

        self._lock = threading.Lock()
        self._busy = False
        self.get_logger().info(
            f'US align: left={left_topic}  right={right_topic}')

    def _on_left(self, msg: Range):
        self._left = float(msg.range)

    def _on_right(self, msg: Range):
        self._right = float(msg.range)

    def _stop(self):
        self._cmd_pub.publish(Twist())

    def _result(self, cmd_id: str, ok: bool, message: str):
        msg = String()
        msg.data = json.dumps({
            'cmd_id': cmd_id, 'success': ok,
            'message': message, 'mode': 'us'})
        self._result_pub.publish(msg)

    def _on_cmd(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except Exception as exc:
            self._result('', False, str(exc))
            return
        threading.Thread(target=self._run, args=(cmd,), daemon=True).start()

    def _run(self, cmd: dict):
        with self._lock:
            if self._busy:
                self._result(str(cmd.get('cmd_id', '')), False, 'busy')
                return
            self._busy = True

        cmd_id = str(cmd.get('cmd_id', ''))
        tol = float(cmd.get('tolerance_m', self._tol))
        confirm_need = int(cmd.get('confirm_frames', self._confirm_need))
        timeout = float(cmd.get('timeout_sec', self._timeout))
        ok = False
        message = 'timeout'
        confirm = 0
        t0 = time.time()
        try:
            while rclpy.ok() and time.time() - t0 < timeout:
                if self._left is None or self._right is None:
                    time.sleep(0.05)
                    continue

                diff = self._left - self._right  # positive → left further away

                if abs(diff) <= tol:
                    confirm += 1
                    self._stop()
                    if confirm >= confirm_need:
                        ok = True
                        message = (
                            f'aligned  L={self._left:.3f} R={self._right:.3f} '
                            f'diff={diff:.4f}')
                        break
                else:
                    confirm = 0
                    tw = Twist()
                    tw.angular.z = max(-self._wmax,
                                       min(self._wmax, self._kp * diff))
                    self._cmd_pub.publish(tw)
                time.sleep(0.05)
        finally:
            self._stop()
            with self._lock:
                self._busy = False
            self._result(cmd_id, ok, message)


def main(args=None):
    rclpy.init(args=args)
    node = AlignUsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
