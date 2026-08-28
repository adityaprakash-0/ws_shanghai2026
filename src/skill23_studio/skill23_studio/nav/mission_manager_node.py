#!/usr/bin/env python3
"""Mission manager: waypoints, planning, controller enable."""

from __future__ import annotations

import json
import math
import threading
import time
from enum import Enum
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, String

from skill23_studio.nav.node_status import NodeStatusPub
from skill23_studio.nav.utils import quat_from_yaw, yaw_from_quat


class MissionState(Enum):
    IDLE = 'idle'
    RUNNING = 'running'
    WAIT_GOAL = 'wait_goal'
    WAIT_ACTION = 'wait_action'
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
        # Id of the goal currently being pursued. Verdicts from the goal
        # checker are only trusted when they carry this id, otherwise a
        # leftover GOAL_REACHED from the previous waypoint makes the mission
        # skip straight past the next one.
        self.goal_id = 0
        self._reached_id = 0
        self._seen_id = 0
        self._goal_sent_t = 0.0
        self._goal_resends = 0
        self._wait_until = 0.0
        self.status = NodeStatusPub(self, 'mission')

        self.plan_pub = self.create_publisher(PoseStamped, '/custom_nav/plan_request', 10)
        self.current_goal_pub = self.create_publisher(
            PoseStamped, '/custom_nav/current_goal', 10)
        self.status_pub = self.create_publisher(String, '/custom_nav/mission/status', 10)
        self.ctrl_pub = self.create_publisher(
            Bool, '/custom_nav/controller/enable',
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ))
        self.stop_pub = self.create_publisher(Empty, '/custom_nav/mission/stop', 10)
        self._us_pub = self.create_publisher(String, '/custom_nav/align/us', 10)
        self._cobra_pub = self.create_publisher(String, '/custom_nav/align/cobra', 10)
        self._align_cmd_pub = self.create_publisher(String, '/studio/align/cmd', 10)
        self._align_cancel_pub = self.create_publisher(Empty, '/studio/align/cancel', 10)
        self._detect_cmd_pub = self.create_publisher(
            String, '/studio/detect_align/cmd', 10)
        self._detect_cancel_pub = self.create_publisher(
            Empty, '/studio/detect_align/cancel', 10)

        self._align_lock = threading.Lock()
        self._align_pending: Dict[str, threading.Event] = {}
        self._align_results: Dict[str, dict] = {}
        self._align_id_counter = 0

        self.create_subscription(String, '/custom_nav/align/result', self._on_align_result, 10)
        self.create_subscription(String, '/studio/align/result', self._on_align_result, 10)
        self.create_subscription(
            String, '/studio/detect_align/result', self._on_align_result, 10)
        self.create_subscription(PoseStamped, '/custom_nav/goal', self._on_single_goal, 10)
        self.create_subscription(String, '/custom_nav/goals', self._on_goals_json, 10)
        self.create_subscription(Empty, '/custom_nav/mission/start', self._on_start, 10)
        self.create_subscription(Empty, '/custom_nav/mission/stop', self._on_stop, 10)
        self.create_subscription(String, '/custom_nav/goal_status', self._on_goal_status, 10)
        self.create_subscription(String, '/custom_nav/goal_check', self._on_goal_check, 10)
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

    @staticmethod
    def _get_action(wp: dict) -> dict:
        """Return the validated action dict or empty dict."""
        action = (wp or {}).get('action')
        if not isinstance(action, dict):
            return {}
        return action

    @staticmethod
    def _wait_seconds(wp: dict) -> float:
        """Return a safe wait duration; unsupported task types are ignored."""
        action = (wp or {}).get('action')
        if not isinstance(action, dict) or action.get('type') != 'wait':
            return 0.0
        try:
            seconds = float(action.get('seconds', 0.0))
        except (TypeError, ValueError):
            return 0.0
        if not math.isfinite(seconds):
            return 0.0
        return min(3600.0, max(0.0, seconds))

    def _normalise_waypoint(self, wp: object) -> dict:
        if not isinstance(wp, dict):
            raise ValueError('each waypoint must be an object')
        clean = dict(wp)
        action = self._get_action(clean)
        atype = action.get('type', '')
        if atype == 'wait':
            seconds = self._wait_seconds(clean)
            if seconds > 0.0:
                clean['action'] = {'type': 'wait', 'seconds': seconds}
            else:
                clean.pop('action', None)
        elif atype == 'detect_align':
            clean['action'] = {'type': 'detect_align'}
        elif atype == 'align':
            clean['action'] = {'type': 'align'}
        elif atype == 'align_move':
            try:
                dist = float(action.get('distance_m', 0.0) or 0.0)
            except (TypeError, ValueError):
                dist = 0.0
            if math.isfinite(dist) and dist > 0.0:
                clean['action'] = {
                    'type': 'align_move',
                    'distance_m': min(5.0, max(0.03, dist)),
                }
            else:
                clean.pop('action', None)
        else:
            clean.pop('action', None)
        return clean

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
            'wait_remaining': round(max(
                0.0,
                self._wait_until - self.get_clock().now().nanoseconds / 1e9,
            ), 1) if self.state == MissionState.WAIT_ACTION else 0.0,
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
            if isinstance(data, list):
                wps = data
            elif isinstance(data, dict):
                wps = data.get('waypoints', [])
            else:
                wps = []
            if not isinstance(wps, list):
                raise ValueError('waypoints is not a list')
            self.waypoints = [self._normalise_waypoint(wp) for wp in wps]
            self.index = 0
            self.status.set('idle', f'{len(self.waypoints)} waypoint(s) loaded')
            self.get_logger().info(f'Loaded {len(self.waypoints)} waypoint(s)')
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
        self._wait_until = 0.0
        self._goal_resends = 0
        self._send_current_goal()
        self._publish_status()

    def _on_stop(self, _msg: Empty) -> None:
        self._wait_until = 0.0
        try:
            self._align_cancel_pub.publish(Empty())
        except Exception:
            pass
        try:
            self._detect_cancel_pub.publish(Empty())
        except Exception:
            pass
        self._set_state(MissionState.STOPPED, 'mission stopped')
        self.ctrl_pub.publish(Bool(data=False))
        self._publish_status()

    def _on_goal_status(self, msg: String) -> None:
        # Display only — progression is driven by _on_goal_check.
        self.goal_status = msg.data.strip()

    def _on_goal_check(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        self._seen_id = int(data.get('goal_id', 0))
        if self._seen_id != self.goal_id or not self.goal_id:
            return
        if str(data.get('state', '')) == 'GOAL_REACHED':
            self._reached_id = self.goal_id

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
                if self.state in (
                        MissionState.RUNNING, MissionState.WAIT_GOAL,
                        MissionState.RECOVERING):
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
        stamp = goal.header.stamp
        self.goal_id = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        self._reached_id = 0
        self.current_goal_pub.publish(goal)
        self.plan_pub.publish(goal)
        self.ctrl_pub.publish(Bool(data=True))
        self._set_state(
            MissionState.WAIT_GOAL,
            f'wp {self.index + 1}/{len(self.waypoints)} '
            f'({wp.get("x", 0):.2f},{wp.get("y", 0):.2f})')
        self.goal_status = 'NOT_REACHED'
        self._goal_sent_t = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(
            f'Goal sent: wp {self.index + 1}/{len(self.waypoints)} '
            f'({wp.get("x", 0):.2f},{wp.get("y", 0):.2f}) id={self.goal_id}')

    def _advance(self) -> None:
        self._wait_until = 0.0
        self.index += 1
        if self.index >= len(self.waypoints):
            self._set_state(MissionState.DONE, 'all goals reached')
            self.ctrl_pub.publish(Bool(data=False))
        else:
            self._set_state(MissionState.RUNNING, f'next wp {self.index + 1}')
            self._goal_resends = 0
            self._send_current_goal()

    def _on_align_result(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        cmd_id = str(data.get('cmd_id', ''))
        with self._align_lock:
            evt = self._align_pending.pop(cmd_id, None)
            if evt:
                self._align_results[cmd_id] = data
                evt.set()

    def _new_align_id(self) -> str:
        self._align_id_counter += 1
        return f'align_{self._align_id_counter}'

    def _run_align(self, mode: str, timeout: float, wp: Optional[dict] = None) -> tuple:
        """Send an align command and block until the result or timeout."""
        cmd_id = self._new_align_id()
        evt = threading.Event()
        with self._align_lock:
            self._align_pending[cmd_id] = evt
        yaw = float((wp or {}).get('yaw', 0.0) or 0.0)
        payload = {
            'cmd_id': cmd_id,
            'mode': mode,
            'yaw': yaw,
        }
        msg = String()
        if mode == 'detect_align':
            msg.data = json.dumps({'cmd_id': cmd_id, 'mode': 'detect_align'})
            self._detect_cmd_pub.publish(msg)
        elif mode == 'align_move':
            action = self._get_action(wp or {})
            try:
                dist = float(action.get('distance_m', 0.0) or 0.0)
            except (TypeError, ValueError):
                dist = 0.0
            payload['distance_m'] = dist
            msg.data = json.dumps(payload)
            self._align_cmd_pub.publish(msg)
        elif mode == 'align':
            msg.data = json.dumps(payload)
            self._align_cmd_pub.publish(msg)
        elif mode == 'us':
            msg.data = json.dumps({'cmd_id': cmd_id, 'timeout_sec': timeout})
            self._us_pub.publish(msg)
        else:
            msg.data = json.dumps({'cmd_id': cmd_id, 'timeout_sec': timeout})
            self._cobra_pub.publish(msg)
        evt.wait(timeout=timeout * 6.0 + 30.0)
        with self._align_lock:
            result = self._align_results.pop(cmd_id, None)
            self._align_pending.pop(cmd_id, None)
        if result:
            return bool(result.get('success')), str(result.get('message', ''))
        return False, 'align timeout — no result'

    def _on_reached(self) -> None:
        """Stop at the waypoint, perform its optional action, then advance."""
        self.ctrl_pub.publish(Bool(data=False))
        wp = self.waypoints[self.index] if self.waypoints else {}
        action = self._get_action(wp)
        atype = action.get('type', '')

        if atype == 'wait':
            seconds = self._wait_seconds(wp)
            if seconds <= 0.0:
                self._advance()
                return
            self._wait_until = (
                self.get_clock().now().nanoseconds / 1e9 + seconds)
            self.goal_status = 'WAITING'
            self._set_state(
                MissionState.WAIT_ACTION,
                f'wp {self.index + 1}: waiting {seconds:g}s')
            self.get_logger().info(
                f'Waypoint {self.index + 1} reached — waiting {seconds:g}s')
            return

        if atype == 'detect_align':
            self._set_state(
                MissionState.WAIT_ACTION,
                f'wp {self.index + 1}: detect+align')
            self.get_logger().info(
                f'Waypoint {self.index + 1} reached — running detect+align')
            threading.Thread(
                target=self._run_align_and_advance,
                args=('detect_align', wp), daemon=True).start()
            return

        if atype == 'align':
            self._set_state(
                MissionState.WAIT_ACTION,
                f'wp {self.index + 1}: align')
            self.get_logger().info(
                f'Waypoint {self.index + 1} reached — running align')
            threading.Thread(
                target=self._run_align_and_advance,
                args=('align', wp), daemon=True).start()
            return

        if atype == 'align_move':
            self._set_state(
                MissionState.WAIT_ACTION,
                f'wp {self.index + 1}: align+move')
            self.get_logger().info(
                f'Waypoint {self.index + 1} reached — running align+move')
            threading.Thread(
                target=self._run_align_and_advance,
                args=('align_move', wp), daemon=True).start()
            return

        self._advance()

    def _run_align_and_advance(self, mode: str, wp: Optional[dict] = None) -> None:
        timeout = 40.0 if mode == 'align_move' else 25.0
        ok, msg = self._run_align(mode, timeout, wp)
        self.get_logger().info(f'Align {mode} result: ok={ok} {msg}')
        self._advance()

    def _resend_goal_if_unheard(self) -> None:
        """Re-issue the goal if the checker never acknowledged this goal id.

        Without this a goal lost at startup (checker not yet subscribed) would
        leave the mission waiting on a verdict that can never arrive.
        """
        if not self.goal_id or self._seen_id == self.goal_id:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._goal_sent_t < 5.0:
            return
        if self._goal_resends >= 3:
            self.status.set(
                'error', 'goal checker never acknowledged the current goal')
            return
        self._goal_resends += 1
        self.get_logger().warn(
            f'Goal checker has not acknowledged goal {self.goal_id} — '
            f'resending (attempt {self._goal_resends})')
        self._send_current_goal()

    def _tick(self) -> None:
        if self.state in (MissionState.RUNNING, MissionState.WAIT_GOAL):
            if self.goal_id and self._reached_id == self.goal_id:
                self._on_reached()
            else:
                self._resend_goal_if_unheard()
        elif self.state == MissionState.WAIT_ACTION:
            if self._wait_until > 0.0:
                now = self.get_clock().now().nanoseconds / 1e9
                if now >= self._wait_until:
                    self.get_logger().info(
                        f'Waypoint {self.index + 1} wait complete')
                    self._advance()
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
