#!/usr/bin/env python3
"""Black-tape alignment using Cobra reflectance channels.

Subscribes to 4 Float32 channels (cobra line-follower).  When a command
arrives on ``/custom_nav/align/cobra`` the node creeps and steers until
the tape centroid is centred, then stops.
"""

from __future__ import annotations

import json
import threading
import time
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32, String


class AlignCobraNode(Node):
    def __init__(self):
        super().__init__('cn_align_cobra')

        self.declare_parameter(
            'cobra_topics',
            ['/line_follower/ch_0', '/line_follower/ch_1',
             '/line_follower/ch_2', '/line_follower/ch_3'])
        self.declare_parameter('tape_threshold', 1.5)
        self.declare_parameter('confirm_frames', 20)
        self.declare_parameter('timeout_sec', 8.0)
        self.declare_parameter('kp_lat', 0.25)
        self.declare_parameter('kp_yaw', 0.4)
        self.declare_parameter('vmax', 0.12)
        self.declare_parameter('wmax', 0.25)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('robot', 'mecanum')

        topics = list(self.get_parameter('cobra_topics').value)
        self._vals: List[Optional[float]] = [None] * len(topics)
        for i, topic in enumerate(topics):
            self.create_subscription(
                Float32, topic,
                lambda msg, idx=i: self._on_ch(idx, msg), 10)

        self._thr = float(self.get_parameter('tape_threshold').value)
        self._confirm = int(self.get_parameter('confirm_frames').value)
        self._timeout = float(self.get_parameter('timeout_sec').value)
        self._kp_lat = float(self.get_parameter('kp_lat').value)
        self._kp_yaw = float(self.get_parameter('kp_yaw').value)
        self._vmax = float(self.get_parameter('vmax').value)
        self._wmax = float(self.get_parameter('wmax').value)
        self._robot = self.get_parameter('robot').value

        self._cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self._result_pub = self.create_publisher(
            String, '/custom_nav/align/result', 10)
        self.create_subscription(
            String, '/custom_nav/align/cobra', self._on_cmd, 10)

        self._lock = threading.Lock()
        self._busy = False
        self.get_logger().info(f'Cobra align listening on {topics}')

    def _on_ch(self, idx: int, msg: Float32):
        self._vals[idx] = float(msg.data)

    def _stop(self):
        self._cmd_pub.publish(Twist())

    def _result(self, cmd_id: str, ok: bool, message: str):
        msg = String()
        msg.data = json.dumps({
            'cmd_id': cmd_id, 'success': ok,
            'message': message, 'mode': 'cobra'})
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
        thr = float(cmd.get('tape_threshold', self._thr))
        confirm_need = int(cmd.get('confirm_frames', self._confirm))
        timeout = float(cmd.get('timeout_sec', self._timeout))
        ok = False
        message = 'timeout'
        confirm = 0
        t0 = time.time()
        try:
            while rclpy.ok() and time.time() - t0 < timeout:
                if any(v is None for v in self._vals):
                    time.sleep(0.05)
                    continue
                dark = [1.0 if (v is not None and v < thr) else 0.0
                        for v in self._vals]
                n = len(dark)
                weights = [i - (n - 1) / 2.0 for i in range(n)]
                total = sum(dark) + 1e-6
                centroid = sum(w * d for w, d in zip(weights, dark)) / total
                on_tape = total >= 1.0

                if on_tape and abs(centroid) < 0.35:
                    confirm += 1
                    self._stop()
                    if confirm >= confirm_need:
                        ok, message = True, f'aligned frames={confirm}'
                        break
                else:
                    confirm = 0
                    tw = Twist()
                    if on_tape:
                        tw.linear.x = 0.06
                        lat = -self._kp_lat * centroid
                        yaw = -self._kp_yaw * centroid
                        if self._robot in ('allterrain', 'grip'):
                            tw.linear.y = 0.0
                            tw.angular.z = max(-self._wmax,
                                               min(self._wmax, yaw))
                        else:
                            tw.linear.y = max(-self._vmax,
                                              min(self._vmax, lat))
                            tw.angular.z = max(-self._wmax,
                                               min(self._wmax, yaw * 0.5))
                    else:
                        tw.linear.x = 0.05
                        tw.angular.z = 0.08
                    self._cmd_pub.publish(tw)
                time.sleep(0.05)
        finally:
            self._stop()
            with self._lock:
                self._busy = False
            self._result(cmd_id, ok, message)


def main(args=None):
    rclpy.init(args=args)
    node = AlignCobraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
