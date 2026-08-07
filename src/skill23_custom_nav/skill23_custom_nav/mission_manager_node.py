#!/usr/bin/env python3
"""Mission manager: waypoints, planning, controller enable."""

from __future__ import annotations

import json
from enum import Enum
from typing import List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String

from skill23_custom_nav.node_status import NodeStatusPub
from skill23_custom_nav.utils import quat_from_yaw, yaw_from_quat


class MissionState(Enum):
    IDLE = 'idle'
    RUNNING = 'running'
    WAIT_GOAL = 'wait_goal'
    RECOVERING = 'recovering'
    DONE = 'done'
    STOPPED = 'stopped'


class MissionManagerNode(Node):
    def __init__(self) -> None:
        super().__init__('cn_mission')
        self.declare_parameter('map_frame', 'map')

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.state = MissionState.IDLE
        self.waypoints: List[dict] = []
        self.index = 0
        self.goal_status = 'NOT_REACHED'
        self.recovery_state = 'idle'
        self.status = NodeStatusPub(self, 'mission')

        self.plan_pub = self.create_publisher(PoseStamped, '/custom_nav/plan_request', 10)
        self.current_goal_pub = self.create_publisher(
            PoseStamped, '/custom_nav/current_goal', 10)
        self.status_pub = self.create_publisher(String, '/custom_nav/mission/status', 10)
        self.ctrl_pub = self.create_publisher(Bool, '/custom_nav/controller/enable', 10)
        self.stop_pub = self.create_publisher(Empty, '/custom_nav/mission/stop', 10)

        self.create_subscription(PoseStamped, '/custom_nav/goal', self._on_single_goal, 10)
        self.create_subscription(String, '/custom_nav/goals', self._on_goals_json, 10)
        self.create_subscription(Empty, '/custom_nav/mission/start', self._on_start, 10)
        self.create_subscription(Empty, '/custom_nav/mission/stop', self._on_stop, 10)
        self.create_subscription(String, '/custom_nav/goal_status', self._on_goal_status, 10)
        self.create_subscription(String, '/custom_nav/recovery', self._on_recovery, 10)
        self.create_timer(0.2, self._tick)
        self.status.set('idle', 'waiting for start')

    def _set_state(self, state: MissionState, detail: str = '') -> None:
        if state != self.state:
            self.state = state
            self.status.set(state.value, detail or state.value)
        else:
            self.state = state

    def _pose_from_wp(self, wp: dict) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.position.x = float(wp.get('x', 0.0))
        msg.pose.position.y = float(wp.get('y', 0.0))
        yaw = float(wp.get('yaw', 0.0))
        qx, qy, qz, qw = quat_from_yaw(yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def _publish_status(self) -> None:
        msg = String()
        current = None
        if self.waypoints and 0 <= self.index < len(self.waypoints):
            current = self.waypoints[self.index]
        msg.data = json.dumps({
            'state': self.state.value,
            'index': self.index,
            'total': len(self.waypoints),
            'wp_idx': self.index,
            'wp_total': len(self.waypoints),
            'goal_status': self.goal_status,
            'recovery': self.recovery_state,
            'current': current,
        })
        self.status_pub.publish(msg)

    def _on_single_goal(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.waypoints = [{
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'yaw': yaw,
        }]
        self.index = 0
        self.status.set('idle', f'1 goal loaded @ ({msg.pose.position.x:.2f},{msg.pose.position.y:.2f})')
        self._publish_status()

    def _on_goals_json(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            wps = data.get('waypoints', data if isinstance(data, list) else [])
            self.waypoints = list(wps)
            self.index = 0
            self.status.set('idle', f'{len(self.waypoints)} waypoint(s) loaded')
            self._publish_status()
        except Exception as exc:
            self.get_logger().warn(f'Bad goals JSON: {exc}')

    def _on_start(self, _msg: Empty) -> None:
        if not self.waypoints:
            self.get_logger().warn('No waypoints to start')
            self.status.set('idle', 'start ignored — no waypoints')
            return
        self._set_state(MissionState.RUNNING, f'starting wp 1/{len(self.waypoints)}')
        self.index = 0
        self._send_current_goal()
        self._publish_status()

    def _on_stop(self, _msg: Empty) -> None:
        self._set_state(MissionState.STOPPED, 'mission stopped')
        self.ctrl_pub.publish(Bool(data=False))
        self._publish_status()

    def _on_goal_status(self, msg: String) -> None:
        self.goal_status = msg.data.strip()

    def _on_recovery(self, msg: String) -> None:
        try:
            prev = self.recovery_state
            data = json.loads(msg.data)
            self.recovery_state = data.get('state', 'idle')
            active = bool(data.get('active', False)) or self.recovery_state in (
                'backup', 'spin', 'wait', 'replan')
            if self.recovery_state == 'fail':
                self._set_state(MissionState.STOPPED, 'recovery failed — stop')
                self.ctrl_pub.publish(Bool(data=False))
            elif active:
                if self.state in (MissionState.RUNNING, MissionState.WAIT_GOAL, MissionState.RECOVERING):
                    detail = data.get('detail') or data.get('action') or self.recovery_state
                    self._set_state(MissionState.RECOVERING, str(detail))
            elif self.recovery_state == 'idle' and prev != 'idle':
                if self.state == MissionState.RECOVERING and self.waypoints:
                    self._set_state(
                        MissionState.WAIT_GOAL,
                        'recovery done — resume')
                    self.ctrl_pub.publish(Bool(data=True))
        except Exception:
            pass

    def _send_current_goal(self) -> None:
        if not self.waypoints or self.index >= len(self.waypoints):
            return
        wp = self.waypoints[self.index]
        goal = self._pose_from_wp(wp)
        self.current_goal_pub.publish(goal)
        self.plan_pub.publish(goal)
        self.ctrl_pub.publish(Bool(data=True))
        self._set_state(
            MissionState.WAIT_GOAL,
            f'wp {self.index + 1}/{len(self.waypoints)} '
            f'({wp.get("x", 0):.2f},{wp.get("y", 0):.2f})')
        self.goal_status = 'NOT_REACHED'

    def _tick(self) -> None:
        if self.state in (MissionState.RUNNING, MissionState.WAIT_GOAL):
            if self.goal_status == 'GOAL_REACHED':
                self.index += 1
                if self.index >= len(self.waypoints):
                    self._set_state(MissionState.DONE, 'all goals reached')
                    self.ctrl_pub.publish(Bool(data=False))
                else:
                    self._set_state(MissionState.RUNNING, f'next wp {self.index + 1}')
                    self._send_current_goal()
        self._publish_status()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
