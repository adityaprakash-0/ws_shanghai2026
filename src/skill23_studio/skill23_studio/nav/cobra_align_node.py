#!/usr/bin/env python3
"""Cobra align: center on reflectance array after a waypoint.

Holonomic (mecanum/xdrive, allow_strafe=true):
  lateral error → linear.y (strafe) until |err| < tol.

Non-holonomic (allterrain/grip, allow_strafe=false):
  same error → angular.z (yaw) + optional small linear.x approach.
  Cannot true-strafe; use for line/heading centering only.

Idle until /studio/cobra/cmd JSON:
  {cmd_id, mode?: "cobra_align", timeout_sec?}
"""

from __future__ import annotations

import json
import math
import threading
import time
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Empty, Float32, String

from skill23_studio.nav.node_status import NodeStatusPub
from skill23_studio.nav.wp_align_common import result_msg, shape_duty


class CobraAlignNode(Node):
    def __init__(self) -> None:
        super().__init__('cobra_align')
        self.declare_parameter('cmd_topic', '/studio/cobra/cmd')
        self.declare_parameter('result_topic', '/studio/cobra/result')
        self.declare_parameter('cancel_topic', '/studio/cobra/cancel')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('channel_prefix', '/line_follower')
        self.declare_parameter('allow_strafe', False)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('timeout_sec', 20.0)
        self.declare_parameter('confirm_frames', 8)
        self.declare_parameter('error_tol', 0.08)
        self.declare_parameter('kp', 1.2)
        self.declare_parameter('duty_min', 0.10)
        self.declare_parameter('duty_max', 0.28)
        self.declare_parameter('approach_duty', 0.0)
        self.declare_parameter('invert_error', False)
        self.declare_parameter('channel_stale_sec', 0.5)
        self.declare_parameter('hold_after_done_sec', 0.5)
        self.declare_parameter('log_hz', 2.0)

        self._allow_strafe = bool(self.get_parameter('allow_strafe').value)
        self._dt = 1.0 / max(5.0, float(self.get_parameter('control_rate_hz').value))
        self._timeout = max(2.0, float(self.get_parameter('timeout_sec').value))
        self._confirm_need = max(1, int(self.get_parameter('confirm_frames').value))
        self._tol = abs(float(self.get_parameter('error_tol').value))
        self._kp = float(self.get_parameter('kp').value)
        self._dmin = abs(float(self.get_parameter('duty_min').value))
        self._dmax = abs(float(self.get_parameter('duty_max').value))
        self._approach = float(self.get_parameter('approach_duty').value)
        self._invert = bool(self.get_parameter('invert_error').value)
        self._stale = max(0.1, float(self.get_parameter('channel_stale_sec').value))
        self._hold = max(0.0, float(self.get_parameter('hold_after_done_sec').value))
        self._log_period = 1.0 / max(0.5, float(self.get_parameter('log_hz').value))

        prefix = str(self.get_parameter('channel_prefix').value or '/line_follower').rstrip('/')
        self._ch: List[Optional[float]] = [None, None, None, None]
        self._ch_stamp = [0.0, 0.0, 0.0, 0.0]
        for i in range(4):
            self.create_subscription(
                Float32, f'{prefix}/ch_{i}',
                lambda msg, idx=i: self._on_ch(idx, msg),
                qos_profile_sensor_data)

        self._cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter('cmd_vel_topic').value), 10)
        self._result_pub = self.create_publisher(
            String, str(self.get_parameter('result_topic').value), 10)
        self.create_subscription(
            String, str(self.get_parameter('cmd_topic').value), self._on_cmd, 10)
        self.create_subscription(
            Empty, str(self.get_parameter('cancel_topic').value),
            self._on_cancel, 10)

        self._cancel = threading.Event()
        self._job_gen = 0
        self._lock = threading.Lock()
        self.status = NodeStatusPub(self, 'cobra_align')
        self.status.set('idle', 'waiting for cobra align command')
        self.get_logger().info(
            f'cobra_align ready  prefix={prefix}  '
            f'allow_strafe={self._allow_strafe}  tol={self._tol:.3f}  '
            f'duty=[{self._dmin:.2f},{self._dmax:.2f}]  '
            f'timeout={self._timeout:.0f}s')

    def _on_ch(self, idx: int, msg: Float32) -> None:
        self._ch[idx] = float(msg.data)
        self._ch_stamp[idx] = time.time()

    def _on_cancel(self, _msg: Empty) -> None:
        self._cancel.set()
        self._stop()

    def _stop(self) -> None:
        try:
            if rclpy.ok():
                self._cmd_pub.publish(Twist())
        except Exception:
            pass

    def _error(self) -> Optional[float]:
        """Positive → need move toward +Y / CCW (depending on mode)."""
        now = time.time()
        vals: List[float] = []
        for i in range(4):
            if self._ch[i] is None or now - self._ch_stamp[i] > self._stale:
                return None
            vals.append(float(self._ch[i]))
        left = 0.5 * (vals[0] + vals[1])
        right = 0.5 * (vals[2] + vals[3])
        err = right - left
        if self._invert:
            err = -err
        return err

    def _on_cmd(self, msg: String) -> None:
        try:
            cmd = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warn(f'[cobra] bad JSON: {exc}')
            self._result_pub.publish(
                result_msg('', False, 'cobra_align', str(exc)))
            return
        self.get_logger().info(
            f'[cobra] command id={cmd.get("cmd_id", "")} '
            f'strafe={self._allow_strafe}')
        self._job_gen += 1
        gen = self._job_gen
        self._cancel.set()
        threading.Thread(target=self._run, args=(cmd, gen), daemon=True).start()

    def _timeout_of(self, cmd: dict) -> float:
        try:
            raw = float(cmd.get('timeout_sec', 0.0) or 0.0)
        except (TypeError, ValueError):
            raw = 0.0
        if raw > 0.0 and math.isfinite(raw):
            return min(60.0, max(2.0, raw))
        return self._timeout

    def _run(self, cmd: dict, gen: int) -> None:
        time.sleep(0.05)
        self._cancel.clear()
        cmd_id = str(cmd.get('cmd_id') or '')
        timeout = self._timeout_of(cmd)
        t0 = time.time()
        last_log = 0.0
        confirm = 0
        err = 0.0
        ok = False
        message = 'timeout'
        self.status.set('aligning', 'cobra centering')

        while rclpy.ok() and not self._cancel.is_set() and gen == self._job_gen:
            now = time.time()
            if now - t0 >= timeout:
                message = f'timeout err={err:+.3f}'
                break
            sample = self._error()
            if sample is None:
                self._stop()
                if now - last_log >= self._log_period:
                    self.get_logger().warn(
                        '[cobra] waiting for fresh /line_follower/ch_* '
                        '(enable cobra in hw_params?)')
                    last_log = now
                time.sleep(self._dt)
                continue
            err = sample
            if abs(err) <= self._tol:
                confirm += 1
                self._stop()
                if confirm >= self._confirm_need:
                    ok = True
                    message = f'aligned err={err:+.3f}'
                    break
            else:
                confirm = 0
                raw = self._kp * err
                duty = shape_duty(raw, self._dmin, self._dmax)
                tw = Twist()
                if self._allow_strafe:
                    tw.linear.y = duty
                    if abs(self._approach) > 1e-6:
                        tw.linear.x = self._approach
                else:
                    tw.angular.z = duty
                    if abs(self._approach) > 1e-6:
                        tw.linear.x = self._approach
                self._cmd_pub.publish(tw)
                if now - last_log >= self._log_period:
                    axis = 'strafe_y' if self._allow_strafe else 'yaw'
                    self.get_logger().info(
                        f'[cobra] {axis} err={err:+.3f} duty={duty:+.2f} '
                        f'confirm={confirm}/{self._confirm_need}')
                    last_log = now
                    self.status.set('aligning', f'err {err:+.3f}')
            time.sleep(self._dt)

        self._stop()
        if ok and self._hold > 0.0 and gen == self._job_gen and not self._cancel.is_set():
            self.status.set('hold', f'{self._hold:.1f}s')
            until = time.time() + self._hold
            while (rclpy.ok() and not self._cancel.is_set()
                   and gen == self._job_gen and time.time() < until):
                self._stop()
                time.sleep(self._dt)

        if gen != self._job_gen:
            return
        if self._cancel.is_set() and not ok:
            message = 'canceled'
        self.status.set('done' if ok else 'idle', message)
        self._result_pub.publish(
            result_msg(
                cmd_id, ok, 'cobra_align', message,
                error=round(err, 4),
                allow_strafe=self._allow_strafe))
        if ok:
            self.get_logger().info(f'[cobra] {message}')
        else:
            self.get_logger().warn(f'[cobra] {message}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CobraAlignNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cancel.set()
        node._stop()
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
