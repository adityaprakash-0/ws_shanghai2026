#!/usr/bin/env python3
"""Republish front/back US + left/right IR as ordered /pp/dis_data."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float32MultiArray


class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('pp_sensor_fusion')
        self.declare_parameter('front_topic', '/ultrasonic_front/range')
        self.declare_parameter('back_topic', '/ultrasonic_back/range')
        self.declare_parameter('left_topic', '/ir_left/range')
        self.declare_parameter('right_topic', '/ir_right/range')
        self.declare_parameter('out_topic', '/pp/dis_data')
        self.declare_parameter('publish_hz', 10.0)

        self._front = -1.0
        self._back = -1.0
        self._left = -1.0
        self._right = -1.0

        self.create_subscription(Range, self.get_parameter('front_topic').value, self._on_front, 10)
        self.create_subscription(Range, self.get_parameter('back_topic').value, self._on_back, 10)
        self.create_subscription(Range, self.get_parameter('left_topic').value, self._on_left, 10)
        self.create_subscription(Range, self.get_parameter('right_topic').value, self._on_right, 10)
        self._pub = self.create_publisher(Float32MultiArray, self.get_parameter('out_topic').value, 10)
        hz = float(self.get_parameter('publish_hz').value)
        self.create_timer(1.0 / max(hz, 1.0), self._tick)
        self.get_logger().info('PP sensor fusion publishing /pp/dis_data [front,back,left,right] metres')

    def _range_m(self, msg: Range) -> float:
        if not math.isfinite(msg.range):
            return -1.0
        return float(msg.range)

    def _on_front(self, msg: Range):
        self._front = self._range_m(msg)

    def _on_back(self, msg: Range):
        self._back = self._range_m(msg)

    def _on_left(self, msg: Range):
        self._left = self._range_m(msg)

    def _on_right(self, msg: Range):
        self._right = self._range_m(msg)

    def _tick(self):
        out = Float32MultiArray()
        out.data = [self._front, self._back, self._left, self._right]
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
