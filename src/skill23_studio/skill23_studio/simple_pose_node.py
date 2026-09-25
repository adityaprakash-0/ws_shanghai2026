#!/usr/bin/env python3
"""Arena teleop pose — start at (0, 0, 0), track x/y/yaw from encoders + IMU.

Uses the drive node's /wheel/odom (already encoder+IMU with correct geometry):
  - mecanum / xdrive: forward + strafe (y)
  - grip / allterrain: differential (forward only, y stays ~0)

Publishes:
  /robot_pose          geometry_msgs/Pose2D   (x, y, theta=yaw)
  /robot_pose_xyyaw    std_msgs/Float64MultiArray  [x, y, yaw]
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger


def _yaw_from_quat(z: float, w: float) -> float:
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


class SimplePoseNode(Node):
    def __init__(self) -> None:
        super().__init__('simple_pose')
        self.declare_parameter('drive_type', 'diff')  # diff | mecanum | xdrive
        self.declare_parameter('odom_topic', '/wheel/odom')
        self.declare_parameter('pose_topic', '/robot_pose')
        self.declare_parameter('xyyaw_topic', '/robot_pose_xyyaw')
        self.declare_parameter('publish_hz', 20.0)

        self._drive = str(self.get_parameter('drive_type').value).lower().strip()
        odom_topic = str(self.get_parameter('odom_topic').value)
        pose_topic = str(self.get_parameter('pose_topic').value)
        xyyaw_topic = str(self.get_parameter('xyyaw_topic').value)
        hz = float(self.get_parameter('publish_hz').value)

        self._have_origin = False
        self._ox = 0.0
        self._oy = 0.0
        self._oyaw = 0.0
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        self._pose_pub = self.create_publisher(Pose2D, pose_topic, 10)
        self._xyyaw_pub = self.create_publisher(Float64MultiArray, xyyaw_topic, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_service(Trigger, '/robot_pose/reset', self._on_reset)
        period = 1.0 / max(1.0, hz)
        self.create_timer(period, self._publish)

        strafe = self._drive in ('mecanum', 'xdrive')
        self.get_logger().info(
            f'simple_pose ready drive={self._drive} '
            f'strafe={"yes" if strafe else "no"} '
            f'origin=0,0,0 topic={pose_topic}')

    def _on_reset(self, _req, res):
        self._have_origin = False
        self._x = self._y = self._yaw = 0.0
        res.success = True
        res.message = 'pose origin reset to current wheel odom'
        self.get_logger().info(res.message)
        return res

    def _on_odom(self, msg: Odometry) -> None:
        px = float(msg.pose.pose.position.x)
        py = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        yaw = _yaw_from_quat(float(q.z), float(q.w))

        if not self._have_origin:
            self._ox, self._oy, self._oyaw = px, py, yaw
            self._have_origin = True

        dx = px - self._ox
        dy = py - self._oy
        c = math.cos(self._oyaw)
        s = math.sin(self._oyaw)
        # Rotate into start-frame so start pose is (0,0,0)
        self._x = c * dx + s * dy
        self._y = -s * dx + c * dy
        self._yaw = self._wrap(yaw - self._oyaw)

        # Diff-drive robots should not report phantom strafe
        if self._drive in ('diff', 'grip', 'allterrain'):
            self._y = 0.0 if abs(self._y) < 1e-4 else self._y

    def _publish(self) -> None:
        pose = Pose2D()
        pose.x = self._x
        pose.y = self._y
        pose.theta = self._yaw
        self._pose_pub.publish(pose)

        arr = Float64MultiArray()
        arr.data = [self._x, self._y, self._yaw]
        self._xyyaw_pub.publish(arr)

    @staticmethod
    def _wrap(a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimplePoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
