#!/usr/bin/env python3
"""Bench trigger: send align / align+move using IMU heading (or AMCL)."""

from __future__ import annotations

import json
import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from skill23_studio.nav.utils import wrap_angle
from skill23_studio.nav.wp_align_common import yaw_of_quat


class AlignTestTrigger(Node):
    def __init__(self) -> None:
        super().__init__('align_test_trigger')
        self.declare_parameter('mode', 'align')
        self.declare_parameter('yaw_deg', 90.0)
        self.declare_parameter('distance_m', 0.10)
        self.declare_parameter('delay_sec', 4.0)
        self.declare_parameter('auto_send', True)
        self.declare_parameter('timeout_sec', 0.0)
        self.declare_parameter('relative', True)
        self.declare_parameter('pose_source', 'imu')
        self.declare_parameter('pose_topic', '/imu')

        self._mode = str(self.get_parameter('mode').value or 'align').strip().lower()
        if self._mode in ('align+move', 'align_move', 'move'):
            self._mode = 'align_move'
        else:
            self._mode = 'align'
        self._yaw_deg = float(self.get_parameter('yaw_deg').value)
        self._distance = float(self.get_parameter('distance_m').value)
        self._delay = max(0.0, float(self.get_parameter('delay_sec').value))
        self._auto = bool(self.get_parameter('auto_send').value)
        self._relative = bool(self.get_parameter('relative').value)
        timeout = float(self.get_parameter('timeout_sec').value)
        self._timeout = timeout if timeout > 0.0 else (
            40.0 if self._mode == 'align_move' else 20.0)
        self._source = str(self.get_parameter('pose_source').value or 'imu')
        self._topic = str(self.get_parameter('pose_topic').value or '/imu')

        self._yaw: Optional[float] = None
        self._sent = False
        self._t0 = time.time()
        self._wait_log = 0.0
        self._align_pub = self.create_publisher(String, '/studio/align/cmd', 10)
        if self._source == 'imu':
            self.create_subscription(Imu, self._topic, self._on_imu, qos_profile_sensor_data)
        else:
            self.create_subscription(
                PoseWithCovarianceStamped, self._topic, self._on_pose, 10)
        self.create_subscription(String, '/studio/align/result', self._on_result, 10)
        if self._auto:
            self.create_timer(0.2, self._try_send)
        self.get_logger().info(
            f'[align_test] ready mode={self._mode} yaw_deg={self._yaw_deg:.1f} '
            f'relative={self._relative} heading={self._source} {self._topic} '
            + (f'distance_m={self._distance:.3f} ' if self._mode == 'align_move' else '')
            + f'delay={self._delay:.1f}s')

    def _on_imu(self, msg: Imu) -> None:
        q = msg.orientation
        self._yaw = yaw_of_quat(q.x, q.y, q.z, q.w)

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        q = msg.pose.pose.orientation
        self._yaw = yaw_of_quat(q.x, q.y, q.z, q.w)

    def _on_result(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            self.get_logger().info(f'[align_test] result {msg.data}')
            return
        self.get_logger().info(
            f'[align_test] RESULT ok={bool(data.get("success"))} '
            f'mode={data.get("mode")}  {data.get("message")}')

    def _try_send(self) -> None:
        if self._sent:
            return
        now = time.time()
        if now - self._t0 < self._delay:
            return
        if self._yaw is None:
            if now - self._wait_log >= 1.0:
                self.get_logger().warn(
                    f'[align_test] waiting for {self._topic} (IMU/motors up?)')
                self._wait_log = now
            return
        nsub = int(self._align_pub.get_subscription_count())
        if nsub < 1:
            if now - self._wait_log >= 1.0:
                self.get_logger().warn(
                    '[align_test] waiting for subscriber on /studio/align/cmd')
                self._wait_log = now
            return
        self._sent = True
        if self._relative:
            target = wrap_angle(self._yaw + math.radians(self._yaw_deg))
            how = f'relative +{self._yaw_deg:.1f}deg'
        else:
            target = math.radians(self._yaw_deg)
            how = f'absolute {self._yaw_deg:.1f}deg'
        now_deg = math.degrees(self._yaw)
        err = math.degrees(wrap_angle(target - self._yaw))
        cmd_id = f'test_{self._mode}_{int(time.time())}'
        payload = {
            'cmd_id': cmd_id,
            'mode': self._mode,
            'yaw': target,
            'timeout_sec': self._timeout,
        }
        if self._mode == 'align_move':
            payload['distance_m'] = self._distance
        self._align_pub.publish(String(data=json.dumps(payload)))
        extra = (
            f'distance={self._distance:.3f}m  ' if self._mode == 'align_move' else '')
        self.get_logger().info(
            f'[align_test] SEND {self._mode} {how} → {math.degrees(target):.1f}deg '
            f'{extra}imu={now_deg:.1f}deg err={err:+.1f}deg  id={cmd_id}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AlignTestTrigger()
    try:
        rclpy.spin(node)
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
