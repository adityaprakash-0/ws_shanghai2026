#!/usr/bin/env python3
"""Single waypoint task node: Align or Align+Move, then hold before done.

Command (JSON on /studio/align/cmd):
  {cmd_id, mode: "align"|"align_move", yaw, distance_m?, timeout_sec?}

Align: PID yaw to the waypoint heading, then hold_after_done_sec, then result.
Align+Move: same yaw PID, pause_after_align_sec, drive to front-lidar distance,
then hold_after_done_sec, then result.

timeout_sec is only the motion abort deadline (PID). It does not include the
hold after success. /cmd_vel is Titan duty (−1…1).
"""

from __future__ import annotations

import json
import math
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String

from skill23_studio.nav.node_status import NodeStatusPub
from skill23_studio.nav.utils import wrap_angle
from skill23_studio.nav.wp_align_common import (
    Pid, bind_heading_sub, range_at_angle, result_msg, shape_duty, twist_lin,
    twist_yaw,
)


class AlignYawNode(Node):
    def __init__(self) -> None:
        super().__init__('align')
        self.declare_parameter('pose_source', 'amcl')
        self.declare_parameter('pose_topic', '/amcl_pose')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('cmd_topic', '/studio/align/cmd')
        self.declare_parameter('result_topic', '/studio/align/result')
        self.declare_parameter('cancel_topic', '/studio/align/cancel')
        self.declare_parameter('scan_topic', '/scan/front')
        self.declare_parameter('scan_topic_alt', '/scan_front')
        self.declare_parameter('scan_angle_deg', 0.0)
        self.declare_parameter('scan_window_deg', 3.0)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('yaw_tolerance_deg', 2.5)
        self.declare_parameter('confirm_frames', 8)
        self.declare_parameter('timeout_sec', 20.0)
        self.declare_parameter('timeout_align_move_sec', 40.0)
        self.declare_parameter('retries', 2)
        self.declare_parameter('retry_pause_sec', 0.4)
        self.declare_parameter('pause_after_align_sec', 0.5)
        self.declare_parameter('hold_after_done_sec', 2.0)
        self.declare_parameter('yaw_kp', 1.4)
        self.declare_parameter('yaw_ki', 0.05)
        self.declare_parameter('yaw_kd', 0.12)
        self.declare_parameter('yaw_i_limit', 0.4)
        self.declare_parameter('yaw_duty_min', 0.12)
        self.declare_parameter('yaw_duty_max', 0.35)
        self.declare_parameter('angular_sign', 1.0)
        self.declare_parameter('dist_tolerance_m', 0.015)
        self.declare_parameter('dist_confirm_frames', 6)
        self.declare_parameter('lin_kp', 4.0)
        self.declare_parameter('lin_ki', 0.05)
        self.declare_parameter('lin_kd', 0.15)
        self.declare_parameter('lin_i_limit', 0.3)
        self.declare_parameter('forward_duty_min', 0.12)
        self.declare_parameter('forward_duty_max', 0.28)
        self.declare_parameter('backward_duty_min', 0.12)
        self.declare_parameter('backward_duty_max', 0.28)
        self.declare_parameter('linear_sign', 1.0)
        self.declare_parameter('log_hz', 2.0)

        self._yaw_tol = math.radians(
            float(self.get_parameter('yaw_tolerance_deg').value))
        self._yaw_confirm_need = max(1, int(self.get_parameter('confirm_frames').value))
        self._timeout_align = float(self.get_parameter('timeout_sec').value)
        self._timeout_move = float(self.get_parameter('timeout_align_move_sec').value)
        self._retries = max(0, min(5, int(self.get_parameter('retries').value)))
        self._retry_pause = max(0.0, float(self.get_parameter('retry_pause_sec').value))
        self._pause = max(0.0, float(self.get_parameter('pause_after_align_sec').value))
        self._hold = max(0.0, float(self.get_parameter('hold_after_done_sec').value))
        self._dt = 1.0 / max(5.0, float(self.get_parameter('control_rate_hz').value))
        self._wmin = abs(float(self.get_parameter('yaw_duty_min').value))
        self._wmax = abs(float(self.get_parameter('yaw_duty_max').value))
        self._fwd_min = abs(float(self.get_parameter('forward_duty_min').value))
        self._fwd_max = abs(float(self.get_parameter('forward_duty_max').value))
        self._back_min = abs(float(self.get_parameter('backward_duty_min').value))
        self._back_max = abs(float(self.get_parameter('backward_duty_max').value))
        asig = float(self.get_parameter('angular_sign').value)
        lsig = float(self.get_parameter('linear_sign').value)
        self._asign = 1.0 if asig >= 0.0 else -1.0
        self._lsign = 1.0 if lsig >= 0.0 else -1.0
        self._dist_tol = abs(float(self.get_parameter('dist_tolerance_m').value))
        self._dist_confirm_need = max(
            1, int(self.get_parameter('dist_confirm_frames').value))
        self._scan_angle = math.radians(
            float(self.get_parameter('scan_angle_deg').value))
        self._scan_window = float(self.get_parameter('scan_window_deg').value)
        self._log_period = 1.0 / max(0.5, float(self.get_parameter('log_hz').value))
        self._yaw_pid = Pid(
            float(self.get_parameter('yaw_kp').value),
            float(self.get_parameter('yaw_ki').value),
            float(self.get_parameter('yaw_kd').value),
            float(self.get_parameter('yaw_i_limit').value),
        )
        self._lin_pid = Pid(
            float(self.get_parameter('lin_kp').value),
            float(self.get_parameter('lin_ki').value),
            float(self.get_parameter('lin_kd').value),
            float(self.get_parameter('lin_i_limit').value),
        )

        self._yaw: Optional[float] = None
        self._yaw_stamp = 0.0
        self._scan: Optional[LaserScan] = None
        self._scan_stamp = 0.0
        self._lock = threading.Lock()
        self._busy = False
        self._cancel = threading.Event()
        self._job_gen = 0

        scan_topic = str(self.get_parameter('scan_topic').value)
        self._cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter('cmd_vel_topic').value), 10)
        self._result_pub = self.create_publisher(
            String, str(self.get_parameter('result_topic').value), 10)
        self._heading_topic = bind_heading_sub(self, self._on_yaw)
        self.create_subscription(
            LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data)
        alt = str(self.get_parameter('scan_topic_alt').value or '').strip()
        if alt and alt != scan_topic:
            self.create_subscription(
                LaserScan, alt, self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            String, str(self.get_parameter('cmd_topic').value), self._on_cmd, 10)
        self.create_subscription(
            Empty, str(self.get_parameter('cancel_topic').value),
            self._on_cancel, 10)

        self.status = NodeStatusPub(self, 'align')
        self.status.set('idle', 'waiting for align / align+move command')
        src = str(self.get_parameter('pose_source').value)
        self.get_logger().info(
            f'align ready  heading={src} {self._heading_topic}  '
            f'scan={scan_topic} @ {math.degrees(self._scan_angle):.1f}deg  '
            f'yaw_tol={math.degrees(self._yaw_tol):.1f}deg  '
            f'timeout align={self._timeout_align:.0f}s '
            f'align_move={self._timeout_move:.0f}s  '
            f'retries={self._retries}  '
            f'hold_after_done={self._hold:.1f}s  '
            f'yaw PID kp={self._yaw_pid.kp:g} ki={self._yaw_pid.ki:g} '
            f'kd={self._yaw_pid.kd:g} duty=[{self._wmin:.2f},{self._wmax:.2f}]  '
            f'lin PID kp={self._lin_pid.kp:g} fwd=[{self._fwd_min:.2f},{self._fwd_max:.2f}] '
            f'back=[{self._back_min:.2f},{self._back_max:.2f}]  '
            f'dist_tol={self._dist_tol:.3f}m')

    def _on_yaw(self, yaw: float) -> None:
        self._yaw = float(yaw)
        self._yaw_stamp = time.time()

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = msg
        self._scan_stamp = time.time()

    def _on_cancel(self, _msg: Empty) -> None:
        self._cancel.set()
        self._stop()

    def _stop(self) -> None:
        try:
            if rclpy.ok():
                self._cmd_pub.publish(Twist())
        except Exception:
            pass

    def _front_range(self) -> tuple:
        scan = self._scan
        if scan is None or time.time() - self._scan_stamp > 0.6:
            return None, self._scan_angle, 'no /scan/front yet'
        rng, ang = range_at_angle(scan, self._scan_angle, self._scan_window)
        if rng is None:
            return None, ang, f'invalid beam at {math.degrees(ang):.1f}deg'
        return rng, ang, ''

    @staticmethod
    def _mode_of(cmd: dict) -> str:
        raw = str(cmd.get('mode') or '').strip().lower()
        if raw in ('align_move', 'align+move', 'move'):
            return 'align_move'
        if raw == 'align':
            return 'align'
        try:
            dist = float(cmd.get('distance_m', 0.0) or 0.0)
        except (TypeError, ValueError):
            dist = 0.0
        if math.isfinite(dist) and dist > 0.0:
            return 'align_move'
        return 'align'

    def _on_cmd(self, msg: String) -> None:
        try:
            cmd = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warn(f'[align] bad command JSON: {exc}')
            self._result_pub.publish(result_msg('', False, 'align', str(exc)))
            return
        mode = self._mode_of(cmd)
        try:
            yaw = float(cmd.get('yaw', 0.0) or 0.0)
        except (TypeError, ValueError):
            yaw = 0.0
        self.get_logger().info(
            f'[align] command received mode={mode} id={cmd.get("cmd_id", "")} '
            f'yaw={math.degrees(yaw):.1f}deg'
            + (f' distance={cmd.get("distance_m")}m' if mode == 'align_move' else ''))
        self._job_gen += 1
        gen = self._job_gen
        self._cancel.set()
        threading.Thread(target=self._run, args=(cmd, gen), daemon=True).start()

    def _lin_duty(self, err_m: float) -> float:
        raw = self._lsign * self._lin_pid.update(err_m, self._dt)
        if raw >= 0.0:
            return shape_duty(raw, self._fwd_min, self._fwd_max)
        return shape_duty(raw, self._back_min, self._back_max)

    def _timeout_of(self, cmd: dict, mode: str) -> float:
        default = self._timeout_move if mode == 'align_move' else self._timeout_align
        try:
            raw = float(cmd.get('timeout_sec', 0.0) or 0.0)
        except (TypeError, ValueError):
            raw = 0.0
        timeout = raw if raw > 0.0 and math.isfinite(raw) else default
        hi = 90.0 if mode == 'align_move' else 60.0
        return min(hi, max(2.0, timeout))

    def _sleep_or_cancel(self, seconds: float, gen: int) -> bool:
        """Stop and wait. Return False if canceled or superseded."""
        until = time.time() + max(0.0, seconds)
        while rclpy.ok() and not self._cancel.is_set() and gen == self._job_gen:
            self._stop()
            if time.time() >= until:
                return True
            time.sleep(self._dt)
        return False

    def _hold_after_success(self, gen: int, mode: str) -> bool:
        """Sit still after success. Return False if canceled mid-hold."""
        if self._hold <= 0.0:
            return True
        self.status.set('hold', f'{self._hold:.1f}s before next waypoint')
        self.get_logger().info(
            f'[{mode}] holding {self._hold:.1f}s before next waypoint')
        return self._sleep_or_cancel(self._hold, gen)

    def _one_attempt(
            self, mode: str, target_yaw: float, target_m: float,
            timeout: float, gen: int, attempt: int, attempts: int):
        """Run one timeout window. Returns (ok, message, state, err_deg, rng)."""
        self._yaw_pid.reset()
        self._lin_pid.reset()
        t0 = time.time()
        last_log = 0.0
        ok = False
        message = 'timeout'
        err_deg = 0.0
        rng = None
        state = 'aligning'
        confirm = 0
        pause_until = 0.0
        tag = f'{attempt}/{attempts}'
        self.status.set('aligning', f'{tag} yaw {math.degrees(target_yaw):.1f}deg')
        if mode == 'align_move':
            self.get_logger().info(
                f'[align_move] attempt {tag} aligning to '
                f'{math.degrees(target_yaw):.1f}deg then move to {target_m:.3f}m  '
                f'timeout={timeout:.0f}s')
        else:
            self.get_logger().info(
                f'[align] attempt {tag} aligning to '
                f'{math.degrees(target_yaw):.1f}deg  '
                f'tol={math.degrees(self._yaw_tol):.1f}deg  timeout={timeout:.0f}s')
        while rclpy.ok() and not self._cancel.is_set() and gen == self._job_gen:
            now = time.time()
            if now - t0 >= timeout:
                message = (
                    f'timeout state={state} yaw_err={err_deg:.1f}deg'
                    + (f' range={rng if rng is not None else "none"}'
                       if mode == 'align_move' else ''))
                break

            if state == 'aligning':
                yaw = self._yaw
                if yaw is None or now - self._yaw_stamp > 1.0:
                    self._stop()
                    if now - last_log >= self._log_period:
                        self.get_logger().warn(
                            f'[{mode}] waiting for {self._heading_topic}')
                        last_log = now
                    time.sleep(self._dt)
                    continue
                err = wrap_angle(target_yaw - yaw)
                err_deg = math.degrees(err)
                if abs(err) <= self._yaw_tol:
                    confirm += 1
                    self._stop()
                    if confirm >= self._yaw_confirm_need:
                        if mode == 'align':
                            ok = True
                            message = f'aligned yaw_err={err_deg:.1f}deg'
                            break
                        state = 'pause'
                        pause_until = now + self._pause
                        confirm = 0
                        self.get_logger().info(
                            f'[align_move] aligned yaw_err={err_deg:.1f}deg '
                            f'— pause {self._pause:.2f}s then moving')
                        self.status.set('pause', f'aligned {err_deg:.1f}deg')
                else:
                    confirm = 0
                    duty = shape_duty(
                        self._asign * self._yaw_pid.update(err, self._dt),
                        self._wmin, self._wmax)
                    self._cmd_pub.publish(twist_yaw(duty))
                    if now - last_log >= self._log_period:
                        self.get_logger().info(
                            f'[{mode}] aligning yaw_err={err_deg:+.1f}deg '
                            f'duty={duty:+.2f}')
                        last_log = now
                        self.status.set(
                            'aligning',
                            f'{tag} err {err_deg:+.1f}deg duty {duty:+.2f}')
                time.sleep(self._dt)
                continue

            if state == 'pause':
                self._stop()
                if now >= pause_until:
                    state = 'moving'
                    self._lin_pid.reset()
                    confirm = 0
                    peek, pang, note = self._front_range()
                    self.get_logger().info(
                        f'[align_move] moving — target {target_m:.3f}m '
                        f'from {math.degrees(pang):.1f}deg scan  '
                        f'measured={peek if peek is None else f"{peek:.3f}m"}'
                        f'{(" (" + note + ")") if note else ""}')
                    self.status.set('moving', f'target {target_m:.3f}m')
                time.sleep(self._dt)
                continue

            rng, pang, note = self._front_range()
            if rng is None:
                self._stop()
                if now - last_log >= self._log_period:
                    self.get_logger().warn(
                        f'[align_move] no valid front range '
                        f'({note or "inf/nan"} @ {math.degrees(pang):.1f}deg) '
                        '— put a wall in front of the lidar')
                    last_log = now
                time.sleep(self._dt)
                continue
            err_m = rng - target_m
            if abs(err_m) <= self._dist_tol:
                confirm += 1
                self._stop()
                if confirm >= self._dist_confirm_need:
                    ok = True
                    message = (
                        f'aligned+moved yaw_err={err_deg:.1f}deg '
                        f'range={rng:.3f}m target={target_m:.3f}m '
                        f'err={err_m:+.3f}m')
                    break
            else:
                confirm = 0
                duty = self._lin_duty(err_m)
                direction = 'forward' if duty > 0.0 else 'backward'
                self._cmd_pub.publish(twist_lin(duty))
                if now - last_log >= self._log_period:
                    self.get_logger().info(
                        f'[align_move] moving {direction} measured={rng:.3f}m '
                        f'target={target_m:.3f}m err={err_m:+.3f}m '
                        f'duty={duty:+.2f}')
                    last_log = now
                    self.status.set(
                        'moving',
                        f'{direction} {rng:.3f}→{target_m:.3f}m')
            time.sleep(self._dt)

        self._stop()
        return ok, message, state, err_deg, rng

    def _run(self, cmd: dict, gen: int) -> None:
        t_wait = time.time()
        while True:
            with self._lock:
                if not self._busy:
                    if gen != self._job_gen:
                        return
                    self._busy = True
                    self._cancel.clear()
                    break
            if time.time() - t_wait > 2.0:
                self._result_pub.publish(
                    result_msg(str(cmd.get('cmd_id', '')), False, 'align', 'busy'))
                return
            time.sleep(0.02)
        cmd_id = str(cmd.get('cmd_id', ''))
        mode = self._mode_of(cmd)
        try:
            target_yaw = float(cmd.get('yaw', 0.0))
        except (TypeError, ValueError):
            target_yaw = 0.0
        try:
            target_m = float(cmd.get('distance_m', 0.0) or 0.0)
        except (TypeError, ValueError):
            target_m = 0.0
        if not math.isfinite(target_yaw):
            target_yaw = 0.0
        if mode == 'align_move' and (not math.isfinite(target_m) or target_m <= 0.0):
            msg = 'invalid distance_m'
            self.get_logger().warn(f'[align] {msg}')
            self._result_pub.publish(result_msg(cmd_id, False, mode, msg))
            with self._lock:
                self._busy = False
            return

        timeout = self._timeout_of(cmd, mode)
        attempts = 1 + self._retries
        ok = False
        message = 'timeout'
        err_deg = 0.0
        rng = None
        state = 'aligning'
        used = 0
        if mode == 'align_move':
            self.get_logger().info(
                f'[align_move] cmd id={cmd_id} yaw={math.degrees(target_yaw):.1f}deg '
                f'distance={target_m:.3f}m timeout={timeout:.0f}s '
                f'attempts={attempts} (retries={self._retries})')
        else:
            self.get_logger().info(
                f'[align] cmd id={cmd_id} yaw={math.degrees(target_yaw):.1f}deg '
                f'timeout={timeout:.0f}s attempts={attempts} (retries={self._retries})')
        try:
            for i in range(attempts):
                used = i + 1
                if not rclpy.ok() or self._cancel.is_set() or gen != self._job_gen:
                    break
                ok, message, state, err_deg, rng = self._one_attempt(
                    mode, target_yaw, target_m, timeout, gen, used, attempts)
                if ok:
                    break
                if self._cancel.is_set() or gen != self._job_gen:
                    break
                left = attempts - used
                if left <= 0:
                    break
                self._stop()
                self.get_logger().warn(
                    f'[{mode}] {message} — retry {used}/{self._retries} '
                    f'({left} left)')
                self.status.set('retry', f'attempt {used + 1}/{attempts}')
                if not self._sleep_or_cancel(self._retry_pause, gen):
                    break
            if self._cancel.is_set() and not ok:
                message = f'canceled state={state} yaw_err={err_deg:.1f}deg'
            elif not ok:
                message = f'{message} after {used} attempt{"s" if used != 1 else ""}'
            if ok:
                if not self._hold_after_success(gen, mode):
                    ok = False
                    message = f'canceled during hold yaw_err={err_deg:.1f}deg'
        finally:
            self._stop()
            with self._lock:
                self._busy = False
            extra = {
                'yaw_err_deg': round(err_deg, 2),
                'state': state,
                'attempts': used,
                'retries': self._retries,
            }
            if mode == 'align_move':
                extra['range_m'] = (
                    None if not isinstance(rng, float) else round(rng, 3))
                extra['target_m'] = round(target_m, 3)
            else:
                extra['yaw_err_deg'] = round(err_deg, 2)
            if ok:
                self.get_logger().info(f'[{mode}] {message}')
                self.status.set('done', message)
            else:
                self.get_logger().warn(f'[{mode}] {message}')
                self.status.set('idle', message)
            self._result_pub.publish(
                result_msg(cmd_id, ok, mode, message, **extra))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AlignYawNode()
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
