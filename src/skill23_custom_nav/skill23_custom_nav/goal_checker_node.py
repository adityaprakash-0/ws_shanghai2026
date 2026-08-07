#!/usr/bin/env python3
"""Goal reach checker."""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import String

from skill23_custom_nav.node_status import NodeStatusPub
from skill23_custom_nav.utils import pose_cov_to_stamped, wrap_angle, yaw_from_quat


class GoalCheckerNode(Node):
    def __init__(self) -> None:
        super().__init__('cn_goal_checker')
        self.declare_parameter('xy_goal_tolerance', 0.08)
        self.declare_parameter('yaw_goal_tolerance', 0.15)

        self.xy_tol = float(self.get_parameter('xy_goal_tolerance').value)
        self.yaw_tol = float(self.get_parameter('yaw_goal_tolerance').value)

        self.pose: Optional[PoseStamped] = None
        self.goal: Optional[PoseStamped] = None
        self._last = ''
        self.status = NodeStatusPub(self, 'goal_checker')

        self.status_pub = self.create_publisher(String, '/custom_nav/goal_status', 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_pose, 10)
        self.create_subscription(PoseStamped, '/custom_nav/current_goal', self._on_goal, 10)
        self.create_timer(0.1, self._check)
        self.status.set('idle', 'no goal')

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self.pose = pose_cov_to_stamped(msg)

    def _on_goal(self, msg: PoseStamped) -> None:
        self.goal = msg
        self.status.set('tracking', 'new goal')

    def _check(self) -> None:
        status = String()
        detail = ''
        if self.pose is None or self.goal is None:
            status.data = 'NOT_REACHED'
            detail = 'waiting'
        else:
            dx = self.goal.pose.position.x - self.pose.pose.position.x
            dy = self.goal.pose.position.y - self.pose.pose.position.y
            dist = math.hypot(dx, dy)
            q = self.pose.pose.orientation
            yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
            gq = self.goal.pose.orientation
            gyaw = yaw_from_quat(gq.x, gq.y, gq.z, gq.w)
            yaw_err = abs(wrap_angle(gyaw - yaw))
            if dist <= self.xy_tol and yaw_err <= self.yaw_tol:
                status.data = 'GOAL_REACHED'
                detail = f'd={dist:.3f}m yaw_err={yaw_err:.2f}'
            elif dist <= self.xy_tol:
                status.data = 'POSITION_REACHED'
                detail = f'd={dist:.3f}m yaw_err={yaw_err:.2f}'
            else:
                status.data = 'NOT_REACHED'
                detail = f'd={dist:.3f}m'
        self.status_pub.publish(status)
        if status.data != self._last:
            self._last = status.data
            self.status.set(status.data.lower(), detail)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoalCheckerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
