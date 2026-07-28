#!/usr/bin/env python3
"""Play with the light tower via /light_tower/set.

Requires control_server running with light_tower.enabled (e.g. params_mecanum.yaml).

  ros2 launch mypackage light_tower_test.launch.py
  # or, if HW already up:
  ros2 run mypackage light_tower_play

Commands cycle every 3 s, or set LIGHT_TOWER_CMD once via ROS param `once_cmd`.
"""

from __future__ import annotations

import itertools

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from studica_control.srv import SetData

STATES = [
    'red',
    'green',
    'yellow',
    'buzzer',
    'red:blink',
    'green:blink_hw',
    'yellow:2.0',
    'off',
]


class LightTowerPlay(Node):
    def __init__(self):
        super().__init__('light_tower_play')
        self.declare_parameter('once_cmd', '')
        self.declare_parameter('period_sec', 3.0)

        self._cycle = itertools.cycle(STATES)
        self.create_subscription(String, '/light_tower/state', self._on_state, 10)
        self._client = self.create_client(SetData, '/light_tower/set')

        once = self.get_parameter('once_cmd').get_parameter_value().string_value
        period = float(self.get_parameter('period_sec').value)
        self._once_sent = False

        if once:
            self._once_cmd = once
            self._once_timer = self.create_timer(0.5, self._try_once)
        else:
            self.create_timer(period, self._next)
            self.get_logger().info(
                f'Cycling light tower every {period:.1f}s — '
                'or: ros2 service call /light_tower/set studica_control/srv/SetData "{params: green}"')

    def _on_state(self, msg: String):
        self.get_logger().info(f'state: {msg.data}')

    def _send(self, cmd: str):
        if not self._client.service_is_ready():
            self.get_logger().warn('/light_tower/set not ready')
            return False
        req = SetData.Request()
        req.params = cmd
        fut = self._client.call_async(req)
        fut.add_done_callback(
            lambda f, c=cmd: self.get_logger().info(
                f'set "{c}" → {f.result().message if f.result() else "fail"}'))
        return True

    def _try_once(self):
        if self._once_sent:
            return
        if self._send(self._once_cmd):
            self._once_sent = True
            self._once_timer.cancel()

    def _next(self):
        self._send(next(self._cycle))


def main(args=None):
    rclpy.init(args=args)
    node = LightTowerPlay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
