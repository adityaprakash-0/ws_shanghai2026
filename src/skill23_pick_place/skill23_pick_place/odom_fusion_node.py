#!/usr/bin/env python3
"""Publish /pp/odom + TF from existing wheel odometry (or encoder fallback)."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Empty
from tf2_ros import TransformBroadcaster


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class OdomFusionNode(Node):
    """
    Prefer /wheel/odom from drivetrain nodes (already encoder+IMU fused).
    Republish as /pp/odom with frame pp_odom so pick-place stays namespaced.
    """

    def __init__(self):
        super().__init__('pp_odom_fusion')
        self.declare_parameter('source_odom_topic', '/wheel/odom')
        self.declare_parameter('odom_topic', '/pp/odom')
        self.declare_parameter('odom_frame', 'pp_odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._offset_x = 0.0
        self._offset_y = 0.0
        self._offset_yaw = 0.0
        self._have = False
        self._last = None

        src = self.get_parameter('source_odom_topic').value
        out = self.get_parameter('odom_topic').value
        self._pub = self.create_publisher(Odometry, out, 10)
        self._tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, src, self._on_odom, 10)
        self.create_subscription(Empty, '/pp/odom/reset', self._on_reset, 10)
        self.create_subscription(Bool, '/pp/odom/reset_yaw', self._on_reset_yaw, 10)
        self.get_logger().info(f'PP odom fusion: {src} -> {out} (frame {self._odom_frame})')

    def _on_reset(self, _msg: Empty):
        if self._last is None:
            return
        self._offset_x = self._last.pose.pose.position.x
        self._offset_y = self._last.pose.pose.position.y
        q = self._last.pose.pose.orientation
        self._offset_yaw = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)
        self.get_logger().info('PP odom reset to current pose as origin')

    def _on_reset_yaw(self, msg: Bool):
        if self._last is None:
            return
        q = self._last.pose.pose.orientation
        yaw = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)
        # Snap to nearest 90° relative to current offset frame
        local = yaw - self._offset_yaw
        snapped = round(local / (math.pi / 2.0)) * (math.pi / 2.0)
        self._offset_yaw = yaw - snapped
        self.get_logger().info(f'PP yaw snapped (local={math.degrees(snapped):.0f}°)')

    def _on_odom(self, msg: Odometry):
        self._last = msg
        self._have = True
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)
        x = msg.pose.pose.position.x - self._offset_x
        y = msg.pose.pose.position.y - self._offset_y
        # Rotate into offset-yaw frame
        cy, sy = math.cos(-self._offset_yaw), math.sin(-self._offset_yaw)
        lx = cy * x - sy * y
        ly = sy * x + cy * y
        lyaw = yaw - self._offset_yaw

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._odom_frame
        out.child_frame_id = self._base_frame
        out.pose.pose.position.x = lx
        out.pose.pose.position.y = ly
        _, _, qz, qw = yaw_to_quat(lyaw)
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw
        out.twist = msg.twist
        self._pub.publish(out)

        tf = TransformStamped()
        tf.header = out.header
        tf.child_frame_id = self._base_frame
        tf.transform.translation.x = lx
        tf.transform.translation.y = ly
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self._tf.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = OdomFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
