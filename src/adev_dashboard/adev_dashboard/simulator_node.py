#!/usr/bin/env python3
"""ADEV simulation — pose from bridge seeds only (no /cmd_vel / Nav2 controller)."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster


def _yaw_to_quat(yaw: float):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def _quat_to_yaw(qz: float, qw: float) -> float:
    return math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)


class AdevSimulatorNode(Node):
    def __init__(self):
        super().__init__('adev_simulator')
        self.enabled = False
        self.anchor_x = 0.0
        self.anchor_y = 0.0
        self.anchor_yaw = 0.0
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        self.pose_pub = self.create_publisher(PoseStamped, '/adev/sim_pose', 10)
        self.wheel_odom_pub = self.create_publisher(Odometry, '/wheel/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Bool, '/adev/simulation', self._sim_cb, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/adev/sim_seed', self._seed_cb, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self._initial_pose_cb, 10)
        self.create_timer(0.05, self._publish_loop)

        self.get_logger().info(
            'ADEV simulator ready — pose updates via /adev/sim_seed (no cmd_vel)')

    @property
    def map_x(self) -> float:
        return self.anchor_x

    @property
    def map_y(self) -> float:
        return self.anchor_y

    @property
    def map_yaw(self) -> float:
        return self.anchor_yaw

    def set_enabled(self, active: bool):
        active = bool(active)
        if active == self.enabled:
            return
        self.enabled = active
        if active:
            self.get_logger().info(
                f'Simulation ON at ({self.map_x:.2f}, {self.map_y:.2f}) — path-follow only')
            self._sync_outputs()
        else:
            self.get_logger().info(
                f'Simulation OFF — pose kept ({self.map_x:.2f}, {self.map_y:.2f})')

    def _sim_cb(self, msg: Bool):
        self.set_enabled(msg.data)

    def _seed_from_pose(self, x: float, y: float, yaw: float):
        self.anchor_x = float(x)
        self.anchor_y = float(y)
        self.anchor_yaw = float(yaw)
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        if self.enabled:
            self._sync_outputs()

    def _seed_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._seed_from_pose(p.x, p.y, _quat_to_yaw(q.z, q.w))

    def _initial_pose_cb(self, msg: PoseWithCovarianceStamped):
        if not self.enabled:
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._seed_from_pose(p.x, p.y, _quat_to_yaw(q.z, q.w))
        self.get_logger().info(
            f'Sim pose set ({p.x:.2f}, {p.y:.2f}, '
            f'{math.degrees(_quat_to_yaw(q.z, q.w)):.1f}°)')

    def _publish_loop(self):
        if self.enabled:
            self._sync_outputs()

    def _sync_outputs(self):
        self._publish_pose()
        self._publish_wheel_odom()
        self._publish_tf()

    def _publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = self.map_x
        msg.pose.position.y = self.map_y
        _, _, qz, qw = _yaw_to_quat(self.map_yaw)
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.pose_pub.publish(msg)

    def _publish_wheel_odom(self):
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        _, _, qz, qw = _yaw_to_quat(self.odom_yaw)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.01
        self.wheel_odom_pub.publish(odom)

    def _publish_tf(self):
        stamp = self.get_clock().now().to_msg()

        map_odom = TransformStamped()
        map_odom.header.stamp = stamp
        map_odom.header.frame_id = 'map'
        map_odom.child_frame_id = 'odom'
        map_odom.transform.translation.x = self.anchor_x
        map_odom.transform.translation.y = self.anchor_y
        _, _, qz, qw = _yaw_to_quat(self.anchor_yaw)
        map_odom.transform.rotation.z = qz
        map_odom.transform.rotation.w = qw

        odom_base = TransformStamped()
        odom_base.header.stamp = stamp
        odom_base.header.frame_id = 'odom'
        odom_base.child_frame_id = 'base_footprint'
        odom_base.transform.translation.x = self.odom_x
        odom_base.transform.translation.y = self.odom_y
        _, _, qz, qw = _yaw_to_quat(self.odom_yaw)
        odom_base.transform.rotation.z = qz
        odom_base.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform([map_odom, odom_base])


def main(args=None):
    rclpy.init(args=args)
    node = AdevSimulatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    try:
        node.destroy_node()
    except Exception:
        pass
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
