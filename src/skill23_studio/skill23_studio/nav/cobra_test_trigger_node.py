#!/usr/bin/env python3
"""Bench trigger: send one cobra-align command after HW is up."""

from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CobraTestTrigger(Node):
    def __init__(self) -> None:
        super().__init__('cobra_test_trigger')
        self.declare_parameter('delay_sec', 4.0)
        self.declare_parameter('auto_send', True)
        self.declare_parameter('timeout_sec', 20.0)

        self._delay = max(0.0, float(self.get_parameter('delay_sec').value))
        self._auto = bool(self.get_parameter('auto_send').value)
        self._timeout = max(2.0, float(self.get_parameter('timeout_sec').value))
        self._sent = False
        self._t0 = time.time()
        self._wait_log = 0.0
        self._pub = self.create_publisher(String, '/studio/cobra/cmd', 10)
        self.create_subscription(String, '/studio/cobra/result', self._on_result, 10)
        if self._auto:
            self.create_timer(0.2, self._try_send)
        self.get_logger().info(
            f'[cobra_test] ready auto={self._auto} delay={self._delay:.1f}s '
            f'timeout={self._timeout:.0f}s')

    def _on_result(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            self.get_logger().info(f'[cobra_test] result {msg.data}')
            return
        self.get_logger().info(
            f'[cobra_test] RESULT ok={bool(data.get("success"))}  '
            f'{data.get("message")}')

    def _try_send(self) -> None:
        if self._sent:
            return
        now = time.time()
        if now - self._t0 < self._delay:
            return
        nsub = int(self._pub.get_subscription_count())
        if nsub < 1:
            if now - self._wait_log >= 1.0:
                self.get_logger().warn(
                    '[cobra_test] waiting for subscriber on /studio/cobra/cmd')
                self._wait_log = now
            return
        payload = {
            'cmd_id': f'cobra_test_{int(now * 1000)}',
            'mode': 'cobra_align',
            'timeout_sec': self._timeout,
        }
        self._pub.publish(String(data=json.dumps(payload)))
        self._sent = True
        self.get_logger().info(f'[cobra_test] SEND {payload}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CobraTestTrigger()
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
