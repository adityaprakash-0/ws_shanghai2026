#!/usr/bin/env python3
"""Goal reach checker.

Publishes two views of the same verdict:

* ``/custom_nav/goal_status`` — bare state string, for dashboards and nodes
  that only need a coarse hint.
* ``/custom_nav/goal_check`` — JSON carrying the id of the goal the verdict
  belongs to. Consumers that gate mission progression MUST use this topic:
  a bare verdict cannot be distinguished from one left over from the previous
  waypoint, which would make the mission skip the next goal.
"""

from __future__ import annotations

import json
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import String

from skill23_studio.nav.activity import ActivityGate
from skill23_studio.nav.node_status import NodeStatusPub
from skill23_studio.nav.utils import pose_cov_to_stamped, wrap_angle, yaw_from_quat


class GoalCheckerNode(Node):
    def __init__(self) -> None:
        super().__init__('cn_goal_checker')
        self.declare_parameter('xy_goal_tolerance', 0.08)
        self.declare_parameter('yaw_goal_tolerance', 0.08)
        # A loose tolerance accepted only after the robot has had
        # yaw_settle_timeout_sec to converge, so a drive that cannot resolve the
        # last couple of degrees delays the mission instead of stalling it.
        self.declare_parameter('yaw_relaxed_tolerance', 0.20)
        self.declare_parameter('yaw_settle_timeout_sec', 8.0)

        self.xy_tol = float(self.get_parameter('xy_goal_tolerance').value)
        self.yaw_tol = float(self.get_parameter('yaw_goal_tolerance').value)
        self.yaw_tol_relaxed = max(
            self.yaw_tol, float(self.get_parameter('yaw_relaxed_tolerance').value))
        self.yaw_timeout = float(self.get_parameter('yaw_settle_timeout_sec').value)

        self.pose: Optional[PoseStamped] = None
        self.goal: Optional[PoseStamped] = None
        self.goal_id = 0
        self._pos_reached_t: Optional[float] = None
        self._relaxed_warned = False
        self._last = ''
        self.status = NodeStatusPub(self, 'goal_checker')

        self.status_pub = self.create_publisher(String, '/custom_nav/goal_status', 10)
        self.check_pub = self.create_publisher(String, '/custom_nav/goal_check', 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_pose, 10)
        self.create_subscription(PoseStamped, '/custom_nav/current_goal', self._on_goal, 10)
        self.gate = ActivityGate(self, active_hz=10.0, idle_hz=2.0)
        self.create_timer(0.1, self._check)
        self.status.set('idle', 'no goal')

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self.pose = pose_cov_to_stamped(msg)

    def _on_goal(self, msg: PoseStamped) -> None:
        self.goal = msg
        stamp = msg.header.stamp
        self.goal_id = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        self._last = ''
        self._pos_reached_t = None
        self._relaxed_warned = False
        self.status.set('tracking', f'new goal id={self.goal_id}')

    def _check(self) -> None:
        if not self.gate.due():
            return
        status = String()
        detail = ''
        dist = float('nan')
        yaw_err = float('nan')
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
            if dist > self.xy_tol:
                self._pos_reached_t = None
                status.data = 'NOT_REACHED'
                detail = f'd={dist:.3f}m'
            else:
                now = self.get_clock().now().nanoseconds * 1e-9
                if self._pos_reached_t is None:
                    self._pos_reached_t = now
                waited = now - self._pos_reached_t
                tol = self.yaw_tol
                if waited >= self.yaw_timeout:
                    tol = self.yaw_tol_relaxed
                    if not self._relaxed_warned and yaw_err > self.yaw_tol:
                        self._relaxed_warned = True
                        self.get_logger().warn(
                            f'final yaw stuck at {yaw_err:.3f} rad after '
                            f'{waited:.1f}s — accepting relaxed {tol:.2f} rad')
                if yaw_err <= tol:
                    status.data = 'GOAL_REACHED'
                else:
                    status.data = 'POSITION_REACHED'
                detail = f'd={dist:.3f}m yaw_err={yaw_err:.2f} tol={tol:.2f}'
        self.status_pub.publish(status)
        self.check_pub.publish(String(data=json.dumps({
            'state': status.data,
            'goal_id': self.goal_id,
            'dist': None if math.isnan(dist) else round(dist, 4),
            'yaw_err': None if math.isnan(yaw_err) else round(yaw_err, 4),
        })))
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
