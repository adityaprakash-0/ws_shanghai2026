#!/usr/bin/env python3
"""Pick-place mission manager with strict pose-settled arrival gate."""

from __future__ import annotations

import json
import math
import threading
import time
import uuid
from typing import Any, Dict, List, Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String


def normalize(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_of(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)


class MissionManagerNode(Node):
    """
    Typed steps: goto | relative | align | pick | place | wait | reset_odom

    Arrival gate (non-negotiable before align/pick/place after goto):
      1) controller reports done
      2) re-read /pp/odom for N samples
      3) |dx|,|dy| < xy_tol, |dyaw| < yaw_tol, near-zero cmd_vel
      4) only then continue
      5) never open gripper on failed arrival
    """

    def __init__(self):
        super().__init__('pp_mission')
        self.declare_parameter('robot', 'mecanum')
        self.declare_parameter('xy_tol', 0.02)
        self.declare_parameter('yaw_tol', 0.05)
        self.declare_parameter('settle_samples', 8)
        self.declare_parameter('settle_dt', 0.05)
        self.declare_parameter('arrival_timeout_sec', 30.0)
        self.declare_parameter('cmd_vel_topic', '/pp/cmd_vel')
        self.declare_parameter('odom_topic', '/pp/odom')
        self.declare_parameter('vel_settle', 0.02)

        self._robot = self.get_parameter('robot').value
        self._xy_tol = float(self.get_parameter('xy_tol').value)
        self._yaw_tol = float(self.get_parameter('yaw_tol').value)
        self._settle_n = int(self.get_parameter('settle_samples').value)
        self._settle_dt = float(self.get_parameter('settle_dt').value)
        self._arrival_timeout = float(self.get_parameter('arrival_timeout_sec').value)
        self._vel_settle = float(self.get_parameter('vel_settle').value)

        self._odom: Optional[Odometry] = None
        self._last_cmd_vel = Twist()
        self._ranges: List[float] = [0.0, 0.0, 0.0, 0.0]
        self._arm_status: Dict[str, Any] = {}

        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._on_odom, 10)
        self.create_subscription(
            Twist, self.get_parameter('cmd_vel_topic').value, self._on_cmd_vel, 10)
        self.create_subscription(Float32MultiArray, '/pp/dis_data', self._on_ranges, 10)
        self.create_subscription(String, '/pp/manipulator/status', self._on_arm, 10)

        self._goto_pub = self.create_publisher(String, '/pp/nav/goto', 10)
        self._rel_pub = self.create_publisher(String, '/pp/nav/relative', 10)
        self._cobra_pub = self.create_publisher(String, '/pp/align/cobra', 10)
        self._cam_pub = self.create_publisher(String, '/pp/align/camera', 10)
        self._arm_pub = self.create_publisher(String, '/pp/manipulator/cmd', 10)
        self._cmd_stop = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self._status_pub = self.create_publisher(String, '/pp/mission/status', 10)

        self.create_subscription(String, '/pp/nav/result', self._on_nav_result, 10)
        self.create_subscription(String, '/pp/align/result', self._on_align_result, 10)
        self.create_subscription(String, '/pp/manipulator/result', self._on_arm_result, 10)
        self.create_subscription(String, '/pp/mission/cmd', self._on_mission_cmd, 10)

        self._lock = threading.Lock()
        self._pending: Dict[str, threading.Event] = {}
        self._pending_ok: Dict[str, bool] = {}
        self._pending_msg: Dict[str, str] = {}

        self._task: Dict[str, Any] = {'version': 1, 'name': '', 'steps': []}
        self._running = False
        self._abort = False
        self._step_idx = -1
        self._state = 'idle'
        self._message = ''
        self.create_timer(0.25, self._publish_status)
        self.get_logger().info(f'Mission manager ready robot={self._robot}')

    def _on_odom(self, msg: Odometry):
        self._odom = msg

    def _on_cmd_vel(self, msg: Twist):
        self._last_cmd_vel = msg

    def _on_ranges(self, msg: Float32MultiArray):
        self._ranges = list(msg.data)[:4]

    def _on_arm(self, msg: String):
        try:
            self._arm_status = json.loads(msg.data)
        except Exception:
            pass

    def _register_wait(self, cmd_id: str) -> threading.Event:
        ev = threading.Event()
        with self._lock:
            self._pending[cmd_id] = ev
            self._pending_ok[cmd_id] = False
            self._pending_msg[cmd_id] = ''
        return ev

    def _finish_wait(self, cmd_id: str, ok: bool, message: str):
        with self._lock:
            self._pending_ok[cmd_id] = ok
            self._pending_msg[cmd_id] = message
            ev = self._pending.get(cmd_id)
        if ev:
            ev.set()

    def _on_nav_result(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        self._finish_wait(str(data.get('cmd_id', '')), bool(data.get('success')),
                          str(data.get('message', '')))

    def _on_align_result(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        self._finish_wait(str(data.get('cmd_id', '')), bool(data.get('success')),
                          str(data.get('message', '')))

    def _on_arm_result(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        self._finish_wait(str(data.get('cmd_id', '')), bool(data.get('success')),
                          str(data.get('message', '')))

    def _new_id(self, prefix: str) -> str:
        return f'{prefix}_{uuid.uuid4().hex[:8]}'

    def _wait_result(self, cmd_id: str, timeout: float) -> tuple[bool, str]:
        ev = self._pending.get(cmd_id)
        if ev is None:
            return False, 'no wait registered'
        ok_wait = ev.wait(timeout=timeout)
        with self._lock:
            success = self._pending_ok.pop(cmd_id, False)
            message = self._pending_msg.pop(cmd_id, '')
            self._pending.pop(cmd_id, None)
        if not ok_wait:
            return False, 'timeout waiting for result'
        return success, message

    def _pose(self) -> Optional[tuple[float, float, float]]:
        if self._odom is None:
            return None
        p = self._odom.pose.pose.position
        return float(p.x), float(p.y), yaw_of(self._odom)

    def _cmd_speed(self) -> float:
        t = self._last_cmd_vel
        return abs(t.linear.x) + abs(t.linear.y) + abs(t.angular.z)

    def _arrival_gate(
        self,
        goal_x: float,
        goal_y: float,
        goal_yaw: float,
        xy_tol: float,
        yaw_tol: float,
        ignore_yaw: bool = False,
    ) -> tuple[bool, str]:
        """Strict settle check after controller reports done."""
        t0 = time.time()
        good = 0
        last_reason = 'no odom'
        while rclpy.ok() and not self._abort and time.time() - t0 < 5.0:
            pose = self._pose()
            if pose is None:
                time.sleep(self._settle_dt)
                continue
            x, y, yaw = pose
            dx, dy = abs(x - goal_x), abs(y - goal_y)
            dyaw = abs(normalize(yaw - goal_yaw))
            vel_ok = self._cmd_speed() < self._vel_settle
            xy_ok = dx <= xy_tol and dy <= xy_tol
            yaw_ok = ignore_yaw or dyaw <= yaw_tol
            if xy_ok and yaw_ok and vel_ok:
                good += 1
                if good >= self._settle_n:
                    return True, f'settled dx={dx:.3f} dy={dy:.3f} dyaw={dyaw:.3f}'
            else:
                good = 0
                last_reason = (
                    f'not settled dx={dx:.3f}/{xy_tol} dy={dy:.3f}/{xy_tol} '
                    f'dyaw={dyaw:.3f}/{yaw_tol} vel={self._cmd_speed():.3f}')
            time.sleep(self._settle_dt)
        return False, last_reason

    def _stop(self):
        self._cmd_stop.publish(Twist())

    def _publish_status(self):
        pose = self._pose()
        msg = String()
        msg.data = json.dumps({
            'state': self._state,
            'running': self._running,
            'step_index': self._step_idx,
            'steps': len(self._task.get('steps') or []),
            'name': self._task.get('name', ''),
            'message': self._message,
            'pose': None if pose is None else {
                'x': round(pose[0], 4), 'y': round(pose[1], 4),
                'yaw': round(pose[2], 4)},
            'ranges': self._ranges,
            'arm': self._arm_status,
            'robot': self._robot,
        })
        self._status_pub.publish(msg)

    def _on_mission_cmd(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().error(f'bad mission cmd: {exc}')
            return
        action = str(cmd.get('action', '')).lower()
        if action == 'set_task':
            task = cmd.get('task') or {}
            self._task = {
                'version': int(task.get('version', 1)),
                'name': str(task.get('name', '')),
                'steps': list(task.get('steps') or []),
            }
            self._state = 'ready'
            self._message = f'loaded {len(self._task["steps"])} steps'
            self.get_logger().info(self._message)
        elif action == 'start':
            if self._running:
                self._message = 'already running'
                return
            if not self._task.get('steps'):
                self._message = 'no steps'
                self._state = 'error'
                return
            self._abort = False
            threading.Thread(target=self._run_mission, daemon=True).start()
        elif action == 'stop':
            self._abort = True
            self._stop()
            # Abort pending waits
            with self._lock:
                for cid, ev in list(self._pending.items()):
                    self._pending_ok[cid] = False
                    self._pending_msg[cid] = 'aborted'
                    ev.set()
            self._state = 'stopped'
            self._message = 'stopped'
            self._running = False
        else:
            self._message = f'unknown action {action}'

    def _run_mission(self):
        self._running = True
        self._state = 'running'
        self._message = 'started'
        steps: List[dict] = list(self._task.get('steps') or [])
        ok_all = True
        try:
            for i, step in enumerate(steps):
                if self._abort:
                    ok_all = False
                    self._message = 'aborted'
                    break
                self._step_idx = i
                stype = str(step.get('type', '')).lower()
                self._message = f'step {i}: {stype}'
                self.get_logger().info(self._message)
                ok, msg = self._run_step(step)
                self._message = msg
                if not ok:
                    ok_all = False
                    policy = str(step.get('on_fail', 'abort')).lower()
                    if policy == 'retry':
                        self.get_logger().warn(f'retry step {i}: {msg}')
                        ok, msg = self._run_step(step)
                        self._message = msg
                        if ok:
                            continue
                    self.get_logger().error(f'step {i} failed: {msg}')
                    break
            else:
                self._message = 'mission complete'
                self._state = 'done'
                return
            self._state = 'error' if not ok_all else 'done'
        except Exception as exc:
            self._state = 'error'
            self._message = str(exc)
            self.get_logger().error(self._message)
        finally:
            self._stop()
            self._running = False
            self._step_idx = -1 if self._state in ('done',) else self._step_idx

    def _run_step(self, step: dict) -> tuple[bool, str]:
        stype = str(step.get('type', '')).lower()
        if stype == 'goto':
            return self._step_goto(step)
        if stype == 'relative':
            return self._step_relative(step)
        if stype == 'align':
            return self._step_align(step)
        if stype in ('pick', 'place'):
            return self._step_manip(step, stype)
        if stype == 'wait':
            sec = float(step.get('sec', step.get('timeout_sec', 1.0)))
            time.sleep(max(0.0, sec))
            return True, f'waited {sec}s'
        if stype == 'reset_odom':
            # Published via bridge or external; mission just notes it
            from std_msgs.msg import Empty
            if not hasattr(self, '_reset_pub'):
                self._reset_pub = self.create_publisher(Empty, '/pp/odom/reset', 10)
            self._reset_pub.publish(Empty())
            time.sleep(0.2)
            return True, 'odom reset'
        return False, f'unknown step type {stype}'

    def _step_goto(self, step: dict) -> tuple[bool, str]:
        x = float(step['x'])
        y = float(step['y'])
        yaw = float(step.get('yaw', 0.0))
        xy_tol = float(step.get('xy_tol', self._xy_tol))
        yaw_tol = float(step.get('yaw_tol', self._yaw_tol))
        ignore_yaw = bool(step.get('ignore_yaw', False))
        timeout = float(step.get('timeout_sec', self._arrival_timeout))
        cmd_id = self._new_id('goto')
        self._register_wait(cmd_id)
        payload = String()
        payload.data = json.dumps({
            'cmd_id': cmd_id,
            'x': x, 'y': y, 'yaw': yaw,
            'xy_tol': xy_tol, 'yaw_tol': yaw_tol,
            'ignore_yaw': ignore_yaw,
        })
        self._goto_pub.publish(payload)
        ok, msg = self._wait_result(cmd_id, timeout + 5.0)
        if not ok:
            self._stop()
            return False, f'goto controller failed: {msg}'
        # Strict arrival gate
        settled, smsg = self._arrival_gate(x, y, yaw, xy_tol, yaw_tol, ignore_yaw)
        if not settled:
            self._stop()
            return False, f'arrival gate failed: {smsg}'
        return True, f'goto ok; {smsg}'

    def _step_relative(self, step: dict) -> tuple[bool, str]:
        cmd_id = self._new_id('rel')
        self._register_wait(cmd_id)
        timeout = float(step.get('timeout_sec', 20.0))
        payload = String()
        payload.data = json.dumps({
            'cmd_id': cmd_id,
            'dx': float(step.get('dx', 0.0)),
            'dy': float(step.get('dy', 0.0)),
            'dyaw': float(step.get('dyaw', step.get('yaw', 0.0))),
        })
        self._rel_pub.publish(payload)
        ok, msg = self._wait_result(cmd_id, timeout + 5.0)
        self._stop()
        return ok, msg

    def _step_align(self, step: dict) -> tuple[bool, str]:
        mode = str(step.get('mode', 'cobra')).lower()
        cmd_id = self._new_id('align')
        self._register_wait(cmd_id)
        timeout = float(step.get('timeout_sec', 8.0))
        payload = String()
        if mode == 'camera':
            payload.data = json.dumps({
                'cmd_id': cmd_id,
                'colors': step.get('colors') or ['red'],
                'timeout_sec': timeout,
                'confirm_frames': int(step.get('confirm_frames', 10)),
            })
            self._cam_pub.publish(payload)
        else:
            payload.data = json.dumps({
                'cmd_id': cmd_id,
                'timeout_sec': timeout,
                'confirm_frames': int(step.get('confirm_frames', 20)),
                'tape_threshold': float(step.get('tape_threshold', 1.5)),
            })
            self._cobra_pub.publish(payload)
        ok, msg = self._wait_result(cmd_id, timeout + 5.0)
        self._stop()
        return ok, msg

    def _step_manip(self, step: dict, action: str) -> tuple[bool, str]:
        # Safety: require recent settled pose (odom present, near-zero cmd)
        if self._odom is None:
            return False, 'no odom — refuse manipulate'
        if self._cmd_speed() > self._vel_settle * 2:
            self._stop()
            time.sleep(0.2)
            if self._cmd_speed() > self._vel_settle * 2:
                return False, 'robot still moving — refuse manipulate'

        cmd_id = self._new_id(action)
        self._register_wait(cmd_id)
        timeout = float(step.get('timeout_sec', 20.0))
        body = {
            'cmd_id': cmd_id,
            'action': action,
            'elevator': step.get('elevator', 'down'),
            'extend': step.get('extend', 'extend'),
            'gripper': step.get(
                'gripper', 'close' if action == 'pick' else 'open'),
        }
        # Optional lift after grip
        if 'elevator_after' in step:
            body['elevator_after'] = step['elevator_after']
        payload = String()
        payload.data = json.dumps(body)
        self._arm_pub.publish(payload)
        ok, msg = self._wait_result(cmd_id, timeout + 5.0)
        if ok and step.get('elevator_after') is not None:
            # Second elevator move if manipulator didn't handle it
            cmd_id2 = self._new_id('elev')
            self._register_wait(cmd_id2)
            p2 = String()
            p2.data = json.dumps({
                'cmd_id': cmd_id2,
                'action': 'elevator',
                'elevator': step['elevator_after'],
            })
            self._arm_pub.publish(p2)
            ok2, msg2 = self._wait_result(cmd_id2, timeout)
            return ok2, f'{msg}; after={msg2}'
        return ok, msg


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
