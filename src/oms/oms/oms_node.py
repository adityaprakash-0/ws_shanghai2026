#!/usr/bin/env python3
"""OMS — one ROS node that drives every mechanism.

Replaces the old six-process layout (oms_rotation, oms_lift, oms_linear,
oms_vertical, oms_rotation_servo, oms_gripper, oms_home) with a single
``oms`` node. Every topic name, JSON payload and oms.yaml key is unchanged,
so Skill23 Studio, detect_align and the teleop helper keep working.

Mechanisms (each owns a callback group so axes still run in parallel):
  rotation        Titan1 M2 duty PID, home = M0 limit_rev
  lift            Titan1 M1 duty PID, up = M1 limit_fwd, floor = M1 limit_rev
  linear          Smart Servo 200 speed PID, Cypher M3, home = M0 limit_fwd
  vertical        standard servo, two poses
  rotation_servo  standard servo, two poses
  gripper         standard servo, two poses
  home            boot / on-demand sequence over the axes above

Config comes from oms.yaml. The per-mechanism section names (oms_rotation,
oms_lift, …) are kept exactly as before and are read directly from the file
rather than through --params-file, because one node cannot claim six
parameter namespaces. Studio's Save (config_io) therefore still works.

  ros2 launch oms oms.launch.py
  ros2 topic pub --once /oms/linear/set_pos std_msgs/msg/Float64 '{data: 80.0}'
"""

from __future__ import annotations

import json
import math
import os
import statistics
import threading
import time
from pathlib import Path
from typing import Any, Optional

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String
from studica_control.srv import SetData

from oms.config_io import load_oms_yaml
from oms.kinematics import (
    lift_chain_mm_per_sprocket_rev,
    lift_mm_per_tick,
    lift_ticks_per_sprocket_rev,
    ticks_per_lift_mm,
    ticks_per_linear_mm,
    ticks_per_oms_deg,
)
from oms.param_util import declare_bool, declare_str

HOME_STEPS = (
    'rotation', 'lift', 'linear', 'vertical', 'rotation_servo', 'gripper')


def _studio_log_path(name: str) -> Path:
    for root in (
            Path('/home/vmx/skill23_ws/studio_maps/_logs'),
            Path(os.path.expanduser('~/skill23_ws/studio_maps/_logs'))):
        try:
            root.mkdir(parents=True, exist_ok=True)
            return root / name
        except OSError:
            continue
    return Path('/tmp') / name


def _shape(raw: float, umin: float, umax: float) -> float:
    """Clamp |raw| into [umin, umax] keeping the sign; 0 stays 0."""
    if not math.isfinite(raw) or abs(raw) < 1e-9:
        return 0.0
    mag = max(float(umin), min(float(umax), abs(raw)))
    return math.copysign(mag, raw)


class _Pid:
    def __init__(self, kp: float, ki: float, kd: float, i_limit: float) -> None:
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.i_limit = abs(float(i_limit))
        self._i = 0.0
        self._prev: Optional[float] = None

    def reset(self) -> None:
        self._i = 0.0
        self._prev = None

    def update(self, err: float, dt: float) -> float:
        dt = max(1e-3, float(dt))
        self._i = max(-self.i_limit, min(self.i_limit, self._i + err * dt))
        deriv = 0.0 if self._prev is None else (err - self._prev) / dt
        self._prev = err
        return self.kp * err + self.ki * self._i + self.kd * deriv


class _Cfg:
    """One oms.yaml ros__parameters block, tolerant of int-vs-float."""

    def __init__(self, data: dict, section: str) -> None:
        self.section = section
        raw = (data.get(section) or {}).get('ros__parameters') or {}
        self._p = dict(raw) if isinstance(raw, dict) else {}

    def has(self, key: str) -> bool:
        return key in self._p

    def f(self, key: str, default: float) -> float:
        try:
            return float(self._p[key])
        except (KeyError, TypeError, ValueError):
            return float(default)

    def i(self, key: str, default: int) -> int:
        try:
            return int(float(self._p[key]))
        except (KeyError, TypeError, ValueError):
            return int(default)

    def b(self, key: str, default: bool) -> bool:
        value = self._p.get(key, default)
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def s(self, key: str, default: str) -> str:
        value = self._p.get(key, default)
        return default if value is None else str(value)

    def raw(self, key: str, default: Any = None) -> Any:
        return self._p.get(key, default)


class _Axis:
    """Shared plumbing: state, job generations, status/result publishing."""

    label = 'axis'

    def __init__(self, node: 'OmsNode', cfg: _Cfg) -> None:
        self.node = node
        self.cfg = cfg
        self.group = MutuallyExclusiveCallbackGroup()
        self._state = 'boot'
        self._homed = False
        self._cancel = threading.Event()
        self._job_gen = 0
        self._gen_lock = threading.Lock()

    # --- logging -----------------------------------------------------
    def log(self, level: str, msg: str) -> None:
        self.node.write_log(level, f'[{self.label}] {msg}')

    # --- job bookkeeping --------------------------------------------
    def _begin_job(self) -> int:
        self._cancel.set()
        with self._gen_lock:
            self._job_gen += 1
            gen = self._job_gen
        time.sleep(0.02)
        self._cancel.clear()
        return gen

    def _alive(self, gen: int) -> bool:
        return rclpy.ok() and not self._cancel.is_set() and gen == self._job_gen

    def _sleep(self, sec: float, step: float = 0.02) -> bool:
        until = time.time() + max(0.0, sec)
        while rclpy.ok() and not self._cancel.is_set() and time.time() < until:
            time.sleep(step)
        return rclpy.ok() and not self._cancel.is_set()

    def cancel(self) -> None:
        self._cancel.set()
        with self._gen_lock:
            self._job_gen += 1

    # --- interfaces implemented by each axis ------------------------
    def home_blocking(self, cmd_id: str) -> bool:
        raise NotImplementedError

    def emergency_stop(self) -> None:
        pass


class _EncoderAxis(_Axis):
    """Common code for the two Titan duty axes and the linear servo axis."""

    def __init__(self, node: 'OmsNode', cfg: _Cfg, motion_log_name: str,
                 motion_log_override: str, csv_header: str) -> None:
        super().__init__(node, cfg)
        self._ticks: Optional[float] = None
        self._home_ticks = 0.0
        self._cmd_text = ''
        self._cmd_target: Optional[float] = None
        self._motion_last = 0.0
        self._motion_period = 1.0 / max(2.0, cfg.f('motion_log_hz', 5.0))
        self._motion_lock = threading.Lock()
        self._motion_fp = None
        self._status_last = 0.0
        self._status_hz_active = max(2.0, cfg.f('status_hz_active', 10.0))
        self._status_hz_idle = max(0.5, cfg.f('status_hz_idle', 2.5))
        self._duty_step = max(0.005, cfg.f('duty_step', 0.01))
        raw = (motion_log_override or cfg.s('motion_log', '')).strip()
        self._motion_log = Path(raw) if raw else _studio_log_path(motion_log_name)
        self._csv_header = csv_header
        self._init_motion_log()

    def _init_motion_log(self) -> None:
        try:
            self._motion_log.parent.mkdir(parents=True, exist_ok=True)
            self._motion_fp = open(
                self._motion_log, 'w', encoding='utf-8', buffering=1)
            self._motion_fp.write(self._csv_header)
        except Exception as exc:
            self._motion_fp = None
            self.node.get_logger().error(f'cannot write {self._motion_log}: {exc}')

    def _write_motion(self, event: str, tail: str, force: bool) -> None:
        now = time.time()
        if not force and (now - self._motion_last) < self._motion_period:
            return
        self._motion_last = now
        ticks = '' if self._ticks is None else f'{self._ticks:.1f}'
        target = '' if self._cmd_target is None else f'{self._cmd_target:.1f}'
        cmd = self._cmd_text.replace(',', ' ')
        line = (
            f'{time.strftime("%H:%M:%S")}.{int((now % 1) * 1000):03d},'
            f'{event},{cmd},{target},{ticks},{tail}')
        try:
            with self._motion_lock:
                if self._motion_fp is not None:
                    self._motion_fp.write(line + '\n')
                    if force:
                        self._motion_fp.flush()
        except Exception:
            pass
        if force:
            self.node.get_logger().info(f'[{self.label}_motion] {line}')

    def _should_publish_status(self) -> bool:
        moving = self._state in ('goto', 'homing', 'stall_recover')
        hz = self._status_hz_active if moving else self._status_hz_idle
        now = time.time()
        if now - self._status_last < (1.0 / hz):
            return False
        self._status_last = now
        return True

    def _titan_call(self, client, motor: int, params: str,
                    speed: float = 0.0, timeout: float = 1.5) -> bool:
        if not client.service_is_ready():
            return False
        req = SetData.Request()
        req.params = params
        req.initparams.n_encoder = motor
        req.initparams.speed = float(speed)
        fut = client.call_async(req)
        t0 = time.time()
        while rclpy.ok() and not fut.done() and (time.time() - t0) < timeout:
            time.sleep(0.02)
        return fut.done()

    def _wait_titan(self, client, timeout: float = 30.0) -> bool:
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < timeout:
            if client.wait_for_service(timeout_sec=0.5):
                return True
        return client.service_is_ready()


class RotationAxis(_EncoderAxis):
    """Titan1 M2 turret. Home = M0 limit_rev = 0°, range −90…+90."""

    label = 'oms_rotation'

    def __init__(self, node: 'OmsNode', cfg: _Cfg, motion_log: str) -> None:
        super().__init__(
            node, cfg, 'oms_motion.csv', motion_log,
            'time,event,cmd,target_deg,ticks,angle_deg,duty,'
            'home_sw,homed,state\n')
        self._motor = cfg.i('rotate_motor', 2)
        self._duty_max = max(0.02, min(1.0, abs(cfg.f('duty', 0.10))))
        self._duty_min = max(0.02, min(self._duty_max, abs(cfg.f('duty_min', 0.06))))
        self._home_duty = max(
            self._duty_max, min(1.0, abs(cfg.f('home_duty', 0.20))))
        self._duty_sign = -1.0 if cfg.f('duty_sign', -1.0) < 0.0 else 1.0
        self._ticks_sign = -1.0 if cfg.f('ticks_sign', 1.0) < 0.0 else 1.0
        self._min_deg = cfg.f('min_deg', -90.0)
        self._max_deg = cfg.f('max_deg', 90.0)
        if self._min_deg > self._max_deg:
            self._min_deg, self._max_deg = self._max_deg, self._min_deg
        self._tol = max(0.2, cfg.f('angle_tol_deg', 1.5))
        self._settle = max(0.05, cfg.f('settle_sec', 0.25))
        self._goto_timeout = max(4.0, cfg.f('goto_timeout_sec', 20.0))
        self._home_timeout = max(4.0, cfg.f('home_timeout_sec', 25.0))
        self._stall_sec = max(0.2, cfg.f('stall_sec', 0.70))
        self._stall_ticks = max(1.0, cfg.f('stall_ticks', 6.0))
        self._stall_travel = max(5.0, cfg.f('stall_travel_deg', 25.0))
        self._dt = 1.0 / max(5.0, cfg.f('control_hz', 20.0))
        self._home_search_sign = (
            -1.0 if cfg.f('home_search_sign', 1.0) < 0.0 else 1.0)
        self._pid = _Pid(
            cfg.f('kp', 0.08), cfg.f('ki', 0.0),
            cfg.f('kd', 0.0), cfg.f('i_limit', 0.2))
        measured = cfg.f('ticks_per_deg', 0.0)
        if measured > 0.0:
            self._ticks_per_deg = measured
            self._tpd_source = 'measured'
        else:
            self._ticks_per_deg = ticks_per_oms_deg(
                cfg.f('encoder_cpr', 6.0), cfg.f('quadrature', 4.0),
                cfg.f('gearbox_ratio', 71.2), cfg.f('sun_teeth', 120.0),
                cfg.f('pinion_teeth', 32.0))
            self._tpd_source = 'formula'

        self._angle = 0.0
        self._home_sw = False
        self._last_duty = 0.0

        srv = cfg.s('titan_service', '/titan1/titan_cmd')
        home_topic = cfg.s('home_limit_topic', '/titan1/m_0/limit_rev')
        self._titan = node.create_client(
            SetData, srv, callback_group=self.group)
        self._motor_cmd = node.create_publisher(
            Float64, cfg.s('cmd_topic', '/titan1/m_2/cmd'), 10)
        self._result_pub = node.create_publisher(String, '/oms/rotation/result', 10)
        self._status_pub = node.create_publisher(String, '/oms/rotation/status', 10)
        self._angle_pub = node.create_publisher(
            Float64, '/oms/rotation/angle_deg', 10)
        node.create_subscription(
            Float64, cfg.s('encoder_topic', '/titan1/m_2/encoder'),
            self._on_encoder, 10, callback_group=self.group)
        node.create_subscription(
            Bool, home_topic, self._on_limit, 10, callback_group=self.group)
        node.create_subscription(
            String, '/oms/rotation/cmd', self._on_cmd, 10,
            callback_group=self.group)
        node.create_subscription(
            Float64, '/oms/rotation/set_angle', self._on_set_angle, 10,
            callback_group=self.group)
        node.create_timer(
            0.1, self.publish_status, callback_group=self.group)
        node.get_logger().info(
            f'OMS rotation  motor=titan1 m_{self._motor}  '
            f'ticks/deg={self._ticks_per_deg:.3f} ({self._tpd_source})  '
            f'duty={self._duty_max:.2f}  duty_sign={self._duty_sign:.0f}  '
            f'home={home_topic}  motion_log={self._motion_log}')

    # --- feedback ----------------------------------------------------
    def _on_encoder(self, msg: Float64) -> None:
        self._ticks = float(msg.data)
        if self._homed:
            self._angle = self._ticks_sign * (
                self._ticks - self._home_ticks) / max(1e-6, self._ticks_per_deg)
        if self._state in ('goto', 'homing', 'stall_recover'):
            self._mlog('sample')

    def _on_limit(self, msg: Bool) -> None:
        self._home_sw = bool(msg.data)

    def _angle_now(self) -> float:
        if self._ticks is None or not self._homed:
            return float(self._angle)
        return self._ticks_sign * (
            self._ticks - self._home_ticks) / max(1e-6, self._ticks_per_deg)

    def _mlog(self, event: str, *, force: bool = False) -> None:
        self._write_motion(
            event,
            f'{self._angle_now():.3f},{self._last_duty:.3f},'
            f'{int(self._home_sw)},{int(self._homed)},{self._state}',
            force)

    def publish_status(self) -> None:
        if not self._should_publish_status():
            return
        self._angle = self._angle_now()
        payload = {
            'state': self._state,
            'homed': self._homed,
            'home_switch': self._home_sw,
            'angle_deg': round(self._angle, 3),
            'ticks': None if self._ticks is None else round(self._ticks, 1),
            'duty': round(self._last_duty, 3),
            'min_deg': self._min_deg,
            'max_deg': self._max_deg,
        }
        try:
            self._status_pub.publish(String(data=json.dumps(payload)))
            self._angle_pub.publish(Float64(data=float(self._angle)))
        except Exception:
            pass

    def _pub_result(self, cmd_id: str, ok: bool, message: str) -> None:
        payload = {
            'cmd_id': cmd_id,
            'success': bool(ok),
            'mode': 'rotation',
            'message': message,
            'angle_deg': round(self._angle_now(), 3),
            'homed': self._homed,
        }
        try:
            self._result_pub.publish(String(data=json.dumps(payload)))
        except Exception:
            pass
        self.log('info' if ok else 'warn', f'result ok={ok} {message}')

    # --- motor -------------------------------------------------------
    def _apply_duty(self, duty: float) -> None:
        duty = max(-1.0, min(1.0, float(duty)))
        if abs(duty - self._last_duty) < self._duty_step:
            return
        self._last_duty = duty
        # Titan driver stores /cmd and resends at motor_update_rate_hz — no
        # blocking set_speed service call needed each control tick.
        self._motor_cmd.publish(Float64(data=duty))

    def stop_motor(self) -> None:
        self._last_duty = 0.0
        self._motor_cmd.publish(Float64(data=0.0))

    def emergency_stop(self) -> None:
        try:
            self.stop_motor()
        except Exception:
            pass

    def _mark_home(self) -> None:
        self.stop_motor()
        self._titan_call(self._titan, self._motor, 'reset_encoder', 0.0)
        time.sleep(0.05)
        self._home_ticks = float(self._ticks or 0.0)
        self._angle = 0.0
        self._homed = True
        self._state = 'idle'
        self.log('info', f'homed ticks={self._home_ticks:.1f}')
        self._mlog('homed', force=True)

    def _rezero_pass_home(self) -> None:
        """Reset angle to 0° at the home switch without stopping the motor.

        Used when a goto crosses 0° (e.g. +45 → −70): the limit fires once,
        we rezero to clear encoder drift, then keep chasing the target.
        """
        self._titan_call(self._titan, self._motor, 'reset_encoder', 0.0)
        time.sleep(0.02)
        ticks = float(self._ticks or 0.0)
        if abs(ticks) <= 25.0:
            self._home_ticks = 0.0
        else:
            self._home_ticks = ticks
        self._angle = 0.0
        self._homed = True
        self.log(
            'info',
            f'pass-home rezero ticks={ticks:.1f} home_ticks={self._home_ticks:.1f} '
            f'→ continue to {self._cmd_target}')
        self._mlog('pass_home_rezero', force=True)

    def _drive_until(self, duty: float, *, until_home: bool,
                     timeout: float, gen: int) -> str:
        """Constant duty. Returns home | stall | timeout | cancel."""
        self._apply_duty(duty)
        t0 = time.time()
        last_move = time.time()
        last_ticks = self._ticks
        stall_arm = t0 + 1.2
        while self._alive(gen):
            if until_home and self._home_sw:
                return 'home'
            now = time.time()
            if now - t0 >= timeout:
                return 'timeout'
            ticks = self._ticks
            if ticks is not None and last_ticks is None:
                last_ticks = ticks
                last_move = now
            elif ticks is not None and last_ticks is not None:
                if abs(ticks - last_ticks) >= self._stall_ticks:
                    last_move = now
                    last_ticks = ticks
                elif (
                    now >= stall_arm
                    and abs(duty) >= self._duty_min
                    and (now - last_move) >= self._stall_sec
                ):
                    self.log(
                        'warn',
                        f'stall ticks={ticks:.1f} (no change ≥{self._stall_ticks:.0f} '
                        f'in {self._stall_sec:.2f}s) duty={duty:+.2f}')
                    return 'stall'
            time.sleep(self._dt)
        return 'cancel'

    def _search_home(self, gen: int) -> bool:
        self._state = 'homing'
        self._homed = False
        if self._home_sw:
            self._mark_home()
            return True
        deadline = time.time() + self._home_timeout
        sign = self._home_search_sign
        reversals = 0
        self._cmd_text = 'home'
        self._cmd_target = 0.0
        self.log(
            'info',
            f'home search duty={sign * self._home_duty:+.2f}  '
            f'switch={self._home_sw} ticks={self._ticks}')
        while self._alive(gen):
            remain = deadline - time.time()
            if remain <= 0.0:
                self.stop_motor()
                self._state = 'home_failed'
                self.log('error', 'home timeout — limit switch never pressed')
                return False
            outcome = self._drive_until(
                sign * self._home_duty, until_home=True, timeout=remain, gen=gen)
            if outcome == 'home':
                self._mark_home()
                return True
            if outcome == 'stall':
                reversals += 1
                self.log('warn', f'home search stalled — reverse ({reversals})')
                self.stop_motor()
                if reversals >= 4:
                    self._state = 'home_failed'
                    self.log(
                        'error',
                        'encoder not moving on M2 in either direction — '
                        'check Titan1 M2 / raise home_duty / watch '
                        '/titan1/m_2/encoder and /titan1/m_0/limit_rev')
                    return False
                if not self._sleep(0.15, self._dt):
                    return False
                sign = -sign
                continue
            self.stop_motor()
            if outcome == 'cancel':
                return False
            self._state = 'home_failed'
            return False
        self.stop_motor()
        return False

    def _recover_stall(self, gen: int) -> bool:
        prev = self._last_duty
        self._state = 'stall_recover'
        self.log('warn', 'stall — reverse to home switch')
        self.stop_motor()
        if not self._sleep(0.12, self._dt):
            return False
        if self._home_sw:
            self._mark_home()
            return True
        duty = -math.copysign(self._home_duty, prev or self._home_search_sign)
        if abs(duty) < 1e-6:
            duty = -self._home_search_sign * self._home_duty
        outcome = self._drive_until(
            duty, until_home=True, timeout=self._home_timeout, gen=gen)
        if outcome == 'home':
            self._mark_home()
            return True
        if outcome == 'stall':
            self.log('warn', 'reverse also stalled — flip again')
            self.stop_motor()
            if not self._sleep(0.12, self._dt):
                return False
            outcome = self._drive_until(
                -duty, until_home=True, timeout=self._home_timeout, gen=gen)
            if outcome == 'home':
                self._mark_home()
                return True
        self.stop_motor()
        self._state = 'home_failed'
        return False

    def _goto(self, target: float, gen: int) -> str:
        target = max(self._min_deg, min(self._max_deg, float(target)))
        if not self._homed and not self._search_home(gen):
            return 'home failed'
        self._state = 'goto'
        self._pid.reset()
        t0 = time.time()
        in_tol_t: Optional[float] = None
        last = time.time()
        last_move = time.time()
        stall_arm = t0 + 1.2
        last_ticks = self._ticks
        start_angle = self._angle_now()
        start_ticks = self._ticks
        # Rising-edge detect: if we start already on the switch, wait for
        # release then a later press (crossing 0° mid-move) before rezero.
        limit_latched = bool(self._home_sw)
        self._cmd_text = f'goto {target:+.2f}'
        self._cmd_target = target
        self.log('info', f'goto {target:+.2f}° from {start_angle:+.2f}°')
        self._mlog('goto_start', force=True)
        while self._alive(gen):
            now = time.time()
            if now - t0 >= self._goto_timeout:
                self.stop_motor()
                self._state = 'idle'
                self._mlog('timeout', force=True)
                return 'timeout'
            # Cross / hit home switch → rezero to 0° and keep the command.
            if self._home_sw and not limit_latched:
                self._rezero_pass_home()
                self._pid.reset()
                last_ticks = self._ticks
                last_move = now
                if abs(target) <= self._tol:
                    self.stop_motor()
                    self._state = 'idle'
                    self._mlog('arrived_home', force=True)
                    return 'ok'
            limit_latched = bool(self._home_sw)

            angle = self._angle_now()
            err = target - angle
            dt = max(self._dt, now - last)
            last = now
            if abs(err) <= self._tol:
                self._apply_duty(0.0)
                if in_tol_t is None:
                    in_tol_t = now
                elif now - in_tol_t >= self._settle:
                    self.stop_motor()
                    self._state = 'idle'
                    self._mlog('arrived', force=True)
                    return 'ok'
            else:
                in_tol_t = None
                raw = self._duty_sign * self._pid.update(err, dt)
                floor = self._duty_min
                move_need = abs(target - start_angle)
                if move_need < 15.0:
                    floor = max(self._duty_min, self._duty_max * 0.65)
                duty = _shape(raw, floor, self._duty_max)
                self._apply_duty(duty)
                ticks = self._ticks
                if ticks is not None and last_ticks is None:
                    last_ticks = ticks
                    last_move = now
                elif ticks is not None and last_ticks is not None:
                    if abs(ticks - last_ticks) >= self._stall_ticks:
                        last_move = now
                        last_ticks = ticks
                    elif (
                        now >= stall_arm
                        and abs(duty) >= self._duty_min
                        and (now - last_move) >= self._stall_sec
                    ):
                        traveled = abs(angle - start_angle)
                        self._mlog('stall', force=True)
                        self.log(
                            'warn',
                            f'stall at software {angle:+.1f}° ticks={ticks:.0f} '
                            f'(started {start_angle:+.1f}° ticks={start_ticks}) '
                            f'target {target:+.1f}° traveled={traveled:.1f}°')
                        if traveled >= self._stall_travel:
                            self.stop_motor()
                            self._state = 'idle'
                            self.log(
                                'warn',
                                'mechanical end stop — not rehoming. '
                                'If this pose is physically 90°, measure ticks '
                                'and set ticks_per_deg = abs(ticks)/90 in oms.yaml')
                            self._mlog('at_stop', force=True)
                            return 'at_stop'
                        move_need = abs(target - start_angle)
                        if move_need < 20.0:
                            self.stop_motor()
                            self._state = 'idle'
                            self.log(
                                'warn',
                                f'small-move stall ({move_need:.1f}°) — '
                                f'not rehoming; retry from caller')
                            self._mlog('stall', force=True)
                            return 'stall'
                        if self._recover_stall(gen):
                            self._mlog('rehomed', force=True)
                            return 'rehomed'
                        return 'stall recover failed'
            time.sleep(self._dt)
        self.stop_motor()
        return 'cancel'

    # --- commands ----------------------------------------------------
    def wait_ready(self) -> bool:
        self._state = 'wait_titan'
        if not self._wait_titan(self._titan):
            self._state = 'home_failed'
            self.log('error', 'titan1 service not ready — cannot home')
            return False
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < 10.0:
            if self._ticks is not None:
                break
            time.sleep(0.05)
        if self._ticks is None:
            self.log('warn', 'no /titan1/m_2/encoder yet — homing anyway')
        return True

    def home_blocking(self, cmd_id: str) -> bool:
        gen = self._begin_job()
        try:
            ok = self._search_home(gen)
            self._pub_result(cmd_id, ok, 'homed' if ok else 'home failed')
            return ok
        finally:
            self.stop_motor()

    def _on_set_angle(self, msg: Float64) -> None:
        self._on_cmd(String(data=json.dumps({
            'cmd_id': f'set_{int(time.time() * 1000)}',
            'mode': 'goto',
            'angle_deg': float(msg.data),
        })))

    def _on_cmd(self, msg: String) -> None:
        try:
            cmd = json.loads(msg.data)
        except Exception as exc:
            self._pub_result('', False, f'bad JSON: {exc}')
            return
        if not isinstance(cmd, dict):
            self._pub_result('', False, 'command must be an object')
            return
        mode = str(cmd.get('mode') or 'goto').strip().lower()
        self._cmd_text = f'{mode} {cmd.get("angle_deg", "")}'.strip()
        if mode == 'goto':
            try:
                self._cmd_target = float(cmd.get('angle_deg'))
            except (TypeError, ValueError):
                self._cmd_target = None
        self._mlog('cmd', force=True)
        threading.Thread(target=self._run_cmd, args=(cmd, mode), daemon=True).start()

    def _run_cmd(self, cmd: dict, mode: str) -> None:
        cmd_id = str(cmd.get('cmd_id') or '')
        if mode in ('stop', 'cancel'):
            self.cancel()
            self.stop_motor()
            self._state = 'idle'
            self._pub_result(cmd_id, True, 'stopped')
            return
        if mode == 'home':
            self.home_blocking(cmd_id)
            return
        if mode != 'goto':
            self._pub_result(cmd_id, False, f'unknown mode {mode}')
            return
        try:
            target = float(cmd.get('angle_deg'))
        except (TypeError, ValueError):
            self._pub_result(cmd_id, False, 'angle_deg required')
            return
        gen = self._begin_job()
        try:
            outcome = self._goto(target, gen)
            self._pub_result(cmd_id, outcome == 'ok', outcome)
        finally:
            if gen == self._job_gen and self._state not in ('home_failed',):
                self._state = 'idle' if self._homed else self._state
            self.stop_motor()


class LiftAxis(_EncoderAxis):
    """Titan1 M1 chain lift. 0 mm = up limit, positive mm travels down."""

    label = 'oms_lift'

    def __init__(self, node: 'OmsNode', cfg: _Cfg, motion_log: str) -> None:
        super().__init__(
            node, cfg, 'oms_lift_motion.csv', motion_log,
            'time,event,cmd,target_mm,ticks,pos_mm,duty,'
            'up_sw,down_sw,homed,state\n')
        self._motor = cfg.i('lift_motor', 1)
        # Fixed bang-bang duty per direction (no duty_min ramp).
        self._duty_up = max(0.05, min(1.0, abs(cfg.f('duty', 0.50))))
        self._duty_down = max(0.05, min(1.0, abs(cfg.f('duty_down', 0.70))))
        self._home_duty = max(
            0.05, min(1.0, abs(cfg.f('home_duty', self._duty_up))))
        # +1 → positive error (need down) → +duty = teleop F. Must match yaml.
        self._duty_sign = -1.0 if cfg.f('duty_sign', 1.0) < 0.0 else 1.0
        self._ticks_sign = -1.0 if cfg.f('ticks_sign', 1.0) < 0.0 else 1.0
        self._min_mm = cfg.f('min_mm', 0.0)
        self._max_mm = max(self._min_mm, cfg.f('max_mm', 240.0))
        self._tol = max(0.5, cfg.f('pos_tol_mm', 2.0))
        self._settle = max(0.05, cfg.f('settle_sec', 0.25))
        self._goto_timeout = max(4.0, cfg.f('goto_timeout_sec', 30.0))
        self._home_timeout = max(4.0, cfg.f('home_timeout_sec', 35.0))
        self._stall_sec = max(0.2, cfg.f('stall_sec', 0.70))
        self._stall_ticks = max(1.0, cfg.f('stall_ticks', 6.0))
        self._dt = 1.0 / max(5.0, cfg.f('control_hz', 20.0))
        self._home_search_sign = (
            -1.0 if cfg.f('home_search_sign', -1.0) < 0.0 else 1.0)
        self._pid = _Pid(
            cfg.f('kp', 0.04), cfg.f('ki', 0.0),
            cfg.f('kd', 0.0), cfg.f('i_limit', 0.3))
        sprocket_teeth = max(1.0, cfg.f('sprocket_teeth', 24.0))
        chain_pitch = max(0.1, cfg.f('chain_pitch_mm', 6.35))
        chain_stages = max(1.0, cfg.f('chain_stages', 1.0))
        enc_cpr = cfg.f('encoder_cpr', 6.0)
        quad = cfg.f('quadrature', 4.0)
        gbox = cfg.f('gearbox_ratio', 99.5)
        bp = cfg.f('bevel_pinion_teeth', 13.0)
        bg = cfg.f('bevel_gear_teeth', 26.0)
        sp = cfg.f('spur_pinion_teeth', 32.0)
        sg = cfg.f('spur_gear_teeth', 64.0)
        measured = cfg.f('ticks_per_mm', 0.0)
        if measured > 0.0:
            self._ticks_per_mm = measured
            self._tpm_source = 'measured'
        else:
            self._ticks_per_mm = ticks_per_lift_mm(
                enc_cpr, quad, gbox, sprocket_teeth, chain_pitch, chain_stages,
                bp, bg, sp, sg)
            self._tpm_source = 'pdf'
        self._mm_per_tick = lift_mm_per_tick(
            enc_cpr, quad, gbox, sprocket_teeth, chain_pitch, chain_stages,
            bp, bg, sp, sg)
        self._ticks_per_sprocket_rev = lift_ticks_per_sprocket_rev(
            enc_cpr, quad, gbox, bp, bg, sp, sg)
        self._chain_mm_per_rev = lift_chain_mm_per_sprocket_rev(
            sprocket_teeth, chain_pitch, chain_stages)

        self._pos_mm = 0.0
        self._floor_mm: Optional[float] = None
        self._up_sw = False
        self._down_sw = False
        self._up_limit_handled = False
        self._up_release_t0: Optional[float] = None
        self._up_home_lock = threading.Lock()
        self._home_latch_lock = threading.Lock()
        self._last_duty = 0.0

        srv = cfg.s('titan_service', '/titan1/titan_cmd')
        up_topic = cfg.s('up_limit_topic', '/titan1/m_1/limit_fwd')
        down_topic = cfg.s('down_limit_topic', '/titan1/m_1/limit_rev')
        self._titan = node.create_client(
            SetData, srv, callback_group=self.group)
        self._motor_cmd = node.create_publisher(
            Float64, cfg.s('cmd_topic', '/titan1/m_1/cmd'), 10)
        self._result_pub = node.create_publisher(String, '/oms/lift/result', 10)
        self._status_pub = node.create_publisher(String, '/oms/lift/status', 10)
        self._pos_pub = node.create_publisher(Float64, '/oms/lift/pos_mm', 10)
        node.create_subscription(
            Float64, cfg.s('encoder_topic', '/titan1/m_1/encoder'),
            self._on_encoder, 10, callback_group=self.group)
        node.create_subscription(
            Bool, up_topic, self._on_up_limit, 10, callback_group=self.group)
        node.create_subscription(
            Bool, down_topic, self._on_down_limit, 10, callback_group=self.group)
        node.create_subscription(
            String, '/oms/lift/cmd', self._on_cmd, 10, callback_group=self.group)
        node.create_subscription(
            Float64, '/oms/lift/set_pos', self._on_set_pos, 10,
            callback_group=self.group)
        node.create_timer(0.1, self.publish_status, callback_group=self.group)
        node.get_logger().info(
            f'OMS lift  motor=titan1 m_{self._motor}  '
            f'ticks/mm={self._ticks_per_mm:.3f} ({self._tpm_source})  '
            f'mm/tick={self._mm_per_tick:.6f}  '
            f'T_rev={self._ticks_per_sprocket_rev:.0f}  '
            f'chain=#25 {sprocket_teeth:.0f}T×{chain_stages:.0f}  '
            f'{self._chain_mm_per_rev:.1f}mm/rev  '
            f'duty_up={self._duty_up:.2f}  duty_down={self._duty_down:.2f}  '
            f'duty_sign={self._duty_sign:.0f}  '
            f'up={up_topic}  down={down_topic}  motion_log={self._motion_log}')

    def _lift_duty(self, toward_down: bool,
                   override: Optional[float] = None) -> float:
        """Fixed magnitude per direction; duty_sign unchanged."""
        mag = self._duty_down if toward_down else self._duty_up
        if override is not None and override > 0.0:
            mag = min(1.0, float(override))
        sign = 1.0 if toward_down else -1.0
        return self._duty_sign * sign * mag

    def _pos_from_ticks(self, ticks: float) -> float:
        return self._ticks_sign * (
            float(ticks) - self._home_ticks) / max(1e-6, self._ticks_per_mm)

    def _on_encoder(self, msg: Float64) -> None:
        self._ticks = float(msg.data)
        if (
            self._up_sw and self._homed
            and self._state not in ('goto', 'homing')
        ):
            self._pos_mm = 0.0
            return
        if self._ticks is not None and self._homed:
            self._pos_mm = self._pos_from_ticks(self._ticks)
        if self._state in ('goto', 'homing'):
            self._mlog('sample')

    def _on_up_limit(self, msg: Bool) -> None:
        was = self._up_sw
        self._up_sw = bool(msg.data)
        if self._up_sw:
            self._up_release_t0 = None
            if (
                not was
                and not self._up_limit_handled
                and self._state == 'idle'
            ):
                self._up_limit_handled = True
                threading.Thread(
                    target=self._idle_up_limit_home, daemon=True).start()
        # Release debounced in publish_status.

    def _on_down_limit(self, msg: Bool) -> None:
        was = self._down_sw
        self._down_sw = bool(msg.data)
        if self._down_sw and not was and self._state == 'idle':
            self._mark_floor()

    def _maybe_clear_up_limit_handled(self) -> None:
        if self._up_sw or not self._up_limit_handled:
            self._up_release_t0 = None
            return
        now = time.time()
        if self._up_release_t0 is None:
            self._up_release_t0 = now
            return
        if now - self._up_release_t0 >= 0.35:
            self._up_limit_handled = False
            self._up_release_t0 = None

    def _idle_up_limit_home(self) -> None:
        if not self._up_home_lock.acquire(blocking=False):
            return
        try:
            if self._up_sw and self._state == 'idle':
                self.log('info', 'lift up limit pressed → encoder zero at home')
                self._mark_home()
        finally:
            self._up_home_lock.release()

    def _pos_now(self) -> float:
        if (
            self._up_sw and self._homed
            and self._state not in ('goto', 'homing')
        ):
            return 0.0
        if self._ticks is not None and self._homed:
            return self._pos_from_ticks(self._ticks)
        return float(self._pos_mm)

    def _max_allowed(self) -> float:
        cap = self._max_mm
        if self._floor_mm is not None:
            cap = min(cap, self._floor_mm)
        return max(self._min_mm, cap)

    def _mlog(self, event: str, *, force: bool = False) -> None:
        self._write_motion(
            event,
            f'{self._pos_now():.2f},{self._last_duty:.3f},'
            f'{int(self._up_sw)},{int(self._down_sw)},'
            f'{int(self._homed)},{self._state}',
            force)

    def publish_status(self) -> None:
        self._maybe_clear_up_limit_handled()
        if not self._should_publish_status():
            return
        self._pos_mm = self._pos_now()
        payload = {
            'state': self._state,
            'homed': self._homed,
            'up_switch': self._up_sw,
            'down_switch': self._down_sw,
            'pos_mm': round(self._pos_mm, 2),
            'floor_mm': self._floor_mm,
            'ticks': None if self._ticks is None else round(self._ticks, 1),
            'duty': round(self._last_duty, 3),
            'min_mm': self._min_mm,
            'max_mm': self._max_allowed(),
        }
        try:
            self._status_pub.publish(String(data=json.dumps(payload)))
            self._pos_pub.publish(Float64(data=float(self._pos_mm)))
        except Exception:
            pass

    def _pub_result(self, cmd_id: str, ok: bool, message: str) -> None:
        payload = {
            'cmd_id': cmd_id,
            'success': bool(ok),
            'mode': 'lift',
            'message': message,
            'pos_mm': round(self._pos_now(), 2),
            'homed': self._homed,
        }
        try:
            self._result_pub.publish(String(data=json.dumps(payload)))
        except Exception:
            pass
        self.log('info' if ok else 'warn', f'result ok={ok} {message}')

    def _apply_duty(self, duty: float) -> None:
        duty = max(-1.0, min(1.0, float(duty)))
        # Hard safety: duty_sign>0 → +duty is DOWN. Never drive into a
        # pressed end-stop (UP ⇒ no upward cmd, DOWN ⇒ no downward cmd).
        toward_down = duty * self._duty_sign > 1e-6
        toward_up = duty * self._duty_sign < -1e-6
        if self._up_sw and toward_up:
            duty = 0.0
        if self._down_sw and toward_down:
            duty = 0.0
        if abs(duty - self._last_duty) < self._duty_step:
            return
        self._last_duty = duty
        self._motor_cmd.publish(Float64(data=duty))

    def stop_motor(self) -> None:
        self._last_duty = 0.0
        self._motor_cmd.publish(Float64(data=0.0))

    def emergency_stop(self) -> None:
        try:
            self.stop_motor()
        except Exception:
            pass

    def _latch_home_at_up_limit(self) -> None:
        """Software zero at UP limit. Encoder reset is best-effort."""
        with self._home_latch_lock:
            self.stop_motor()
            if self._titan.service_is_ready():
                self._titan_call(self._titan, self._motor, 'reset_encoder', 0.0)
            samples: list[float] = []
            t0 = time.time()
            while rclpy.ok() and (time.time() - t0) < 0.30:
                if self._ticks is not None:
                    samples.append(float(self._ticks))
                time.sleep(0.02)
            raw = (
                float(statistics.median(samples))
                if samples else float(self._ticks or 0.0))
            self._home_ticks = raw
            self._pos_mm = 0.0
            self._homed = True
            if self._state not in ('goto', 'homing'):
                self._state = 'idle'
            self.log(
                'info',
                f'software home latched raw_ticks={raw:.1f} '
                f'(encoder reset best-effort, pos=0 mm)')
            self._mlog('homed', force=True)

    def _mark_home(self) -> None:
        self._latch_home_at_up_limit()

    def _mark_floor(self) -> None:
        self.stop_motor()
        pos = self._pos_now()
        self._floor_mm = max(self._min_mm, pos)
        self._max_mm = min(self._max_mm, self._floor_mm)
        self.log('info', f'floor limit pos={pos:.1f} mm')
        self._mlog('floor', force=True)

    def _drive_until(self, duty: float, *, until_up: bool, until_down: bool,
                     timeout: float, gen: int) -> str:
        self._apply_duty(duty)
        t0 = time.time()
        last_move = time.time()
        last_ticks = self._ticks
        stall_arm = t0 + 1.2
        while self._alive(gen):
            if until_up and self._up_sw:
                return 'up'
            if until_down and self._down_sw:
                return 'down'
            now = time.time()
            if now - t0 >= timeout:
                return 'timeout'
            ticks = self._ticks
            if ticks is not None and last_ticks is None:
                last_ticks = ticks
                last_move = now
            elif ticks is not None and last_ticks is not None:
                if abs(ticks - last_ticks) >= self._stall_ticks:
                    last_move = now
                    last_ticks = ticks
                elif (
                    now >= stall_arm
                    and abs(duty) >= 0.05
                    and (now - last_move) >= self._stall_sec
                ):
                    return 'stall'
            time.sleep(self._dt)
        return 'cancel'

    def _search_home(self, gen: int) -> bool:
        self._state = 'homing'
        if self._up_sw:
            self._mark_home()
            return True
        sign = self._home_search_sign
        self._cmd_text = 'home'
        self._cmd_target = 0.0
        self.log(
            'info',
            f'home up duty={sign * self._home_duty:+.2f}  '
            f'up_sw={self._up_sw} ticks={self._ticks}')
        outcome = self._drive_until(
            sign * self._home_duty, until_up=True, until_down=False,
            timeout=self._home_timeout, gen=gen)
        if outcome == 'up':
            self._mark_home()
            return True
        self.stop_motor()
        self._state = 'home_failed'
        self.log('error', f'home failed ({outcome})')
        return False

    def _goto(self, target: float, gen: int,
              duty_override: Optional[float] = None) -> str:
        target = max(self._min_mm, min(self._max_allowed(), float(target)))
        if not self._homed and not self._search_home(gen):
            return 'home failed'
        self._state = 'goto'
        self._pid.reset()
        t0 = time.time()
        in_tol_t: Optional[float] = None
        last = time.time()
        self._cmd_text = f'goto {target:.1f}'
        self._cmd_target = target
        self.log(
            'info',
            f'goto {target:.1f} mm from {self._pos_now():.1f} mm  '
            f'duty_sign={self._duty_sign:.0f}  '
            f'up={self._duty_up:.2f}  down={self._duty_down:.2f}  '
            f'lim up={int(self._up_sw)} down={int(self._down_sw)}')
        self._mlog('goto_start', force=True)
        while self._alive(gen):
            now = time.time()
            if now - t0 >= self._goto_timeout:
                self.stop_motor()
                self._state = 'idle'
                self._mlog('timeout', force=True)
                return 'timeout'
            # Arrived at UP home.
            if self._up_sw and target <= self._min_mm + self._tol:
                self._mark_home()
                return 'ok'
            # Floor switch: never drive further down; allow UP leave.
            if self._down_sw:
                self._mark_floor()
            pos = self._pos_now()
            err = target - pos
            dt = max(self._dt, now - last)
            last = now
            if abs(err) <= self._tol:
                self._apply_duty(0.0)
                if in_tol_t is None:
                    in_tol_t = now
                elif now - in_tol_t >= self._settle:
                    self.stop_motor()
                    self._state = 'idle'
                    self._mlog('arrived', force=True)
                    return 'ok'
            else:
                in_tol_t = None
                # Want UP while UP switch pressed → done at home.
                if self._up_sw and err < 0:
                    self.stop_motor()
                    self._mark_home()
                    return 'ok'
                # Want DOWN while floor pressed → stop (hard max).
                if self._down_sw and err > 0:
                    self.stop_motor()
                    self._state = 'idle'
                    self._mlog('at_floor', force=True)
                    return 'at_floor'
                toward_down = (self._duty_sign * err) > 0.0
                # Leave UP switch: force DOWN only (teleop F = +duty).
                if self._up_sw and err > self._tol:
                    toward_down = True
                # Leave floor: force UP only.
                elif self._down_sw and err < -self._tol:
                    toward_down = False
                self._apply_duty(
                    self._lift_duty(toward_down, override=duty_override))
            time.sleep(self._dt)
        self.stop_motor()
        return 'cancel'

    def wait_ready(self) -> bool:
        self._state = 'wait_titan'
        if not self._wait_titan(self._titan):
            self._state = 'home_failed'
            self.log('error', 'titan1 service not ready — cannot home lift')
            return False
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < 10.0:
            if self._ticks is not None:
                break
            time.sleep(0.05)
        return True

    def home_blocking(self, cmd_id: str) -> bool:
        gen = self._begin_job()
        try:
            ok = self._search_home(gen)
            self._pub_result(cmd_id, ok, 'homed up' if ok else 'home failed')
            return ok
        finally:
            self.stop_motor()

    def _on_set_pos(self, msg: Float64) -> None:
        self._on_cmd(String(data=json.dumps({
            'cmd_id': f'set_{int(time.time() * 1000)}',
            'mode': 'goto',
            'pos_mm': float(msg.data),
        })))

    def _on_cmd(self, msg: String) -> None:
        try:
            cmd = json.loads(msg.data)
        except Exception as exc:
            self._pub_result('', False, f'bad JSON: {exc}')
            return
        if not isinstance(cmd, dict):
            self._pub_result('', False, 'command must be an object')
            return
        mode = str(cmd.get('mode') or 'goto').strip().lower()
        self._cmd_text = f'{mode} {cmd.get("pos_mm", "")}'.strip()
        if mode == 'goto':
            try:
                self._cmd_target = float(cmd.get('pos_mm'))
            except (TypeError, ValueError):
                self._cmd_target = None
        self._mlog('cmd', force=True)
        threading.Thread(target=self._run_cmd, args=(cmd, mode), daemon=True).start()

    def _run_cmd(self, cmd: dict, mode: str) -> None:
        cmd_id = str(cmd.get('cmd_id') or '')
        duty_override = None
        if cmd.get('duty') is not None:
            try:
                duty_override = abs(float(cmd['duty']))
            except (TypeError, ValueError):
                duty_override = None
        if mode in ('stop', 'cancel'):
            self.cancel()
            self.stop_motor()
            self._state = 'idle'
            self._pub_result(cmd_id, True, 'stopped')
            return
        if mode in ('home', 'up'):
            self.home_blocking(cmd_id)
            return
        if mode == 'down':
            gen = self._begin_job()
            outcome = self._goto(self._max_allowed(), gen, duty_override=duty_override)
            self._pub_result(cmd_id, outcome in ('ok', 'at_floor'), outcome)
            return
        if mode == 'goto':
            try:
                target = float(cmd.get('pos_mm'))
            except (TypeError, ValueError):
                self._pub_result(cmd_id, False, 'missing pos_mm')
                return
            gen = self._begin_job()
            outcome = self._goto(target, gen, duty_override=duty_override)
            self._pub_result(cmd_id, outcome in ('ok', 'at_floor'), outcome)
            return
        self._pub_result(cmd_id, False, f'unknown mode {mode}')


class LinearAxis(_EncoderAxis):
    """Smart Servo 200 rail with Cypher Max on M3. 0 mm = rear limit."""

    label = 'oms_linear'

    def __init__(self, node: 'OmsNode', cfg: _Cfg, motion_log: str) -> None:
        super().__init__(
            node, cfg, 'oms_linear_motion.csv', motion_log,
            'time,event,cmd,target_mm,ticks,pos_mm,speed,home_sw,homed,state\n')
        self._enc_motor = cfg.i('encoder_motor', 3)
        self._speed_max = max(5.0, min(100.0, abs(cfg.f('speed', 30.0))))
        self._speed_min = max(
            5.0, min(self._speed_max, abs(cfg.f('speed_min', 12.0))))
        # Home may be gentler than cruise (was incorrectly forced ≥ speed_max).
        self._home_speed = max(
            self._speed_min, min(self._speed_max, abs(cfg.f('home_speed', 20.0))))
        self._speed_sign = -1.0 if cfg.f('speed_sign', -1.0) < 0.0 else 1.0
        self._ticks_sign = -1.0 if cfg.f('ticks_sign', 1.0) < 0.0 else 1.0
        self._min_mm = cfg.f('min_mm', 0.0)
        self._max_mm = max(self._min_mm, cfg.f('max_mm', 145.0))
        self._tol = max(0.5, cfg.f('pos_tol_mm', 3.0))
        self._settle = max(0.05, cfg.f('settle_sec', 0.30))
        self._goto_timeout = max(4.0, cfg.f('goto_timeout_sec', 40.0))
        self._home_timeout = max(4.0, cfg.f('home_timeout_sec', 40.0))
        self._stall_sec = max(0.2, cfg.f('stall_sec', 0.80))
        self._stall_ticks = max(1.0, cfg.f('stall_ticks', 6.0))
        self._dt = 1.0 / max(5.0, cfg.f('control_hz', 20.0))
        self._pid = _Pid(
            cfg.f('kp', 0.15), cfg.f('ki', 0.0),
            cfg.f('kd', 0.0), cfg.f('i_limit', 0.4))
        self._enc_max_jump = max(20.0, cfg.f('enc_max_jump_ticks', 80.0))
        self._pos_alpha = max(0.05, min(1.0, cfg.f('pos_filter_alpha', 0.35)))
        self._approach_mm = max(self._tol * 2.0, cfg.f('approach_mm', 8.0))
        # A continuous Smart Servo has a non-zero breakaway command and coasts
        # after stop.  Near a target, sustained +/-speed_min makes the rail hunt
        # past the setpoint.  Use short breakaway pulses and measure after coast.
        self._precision_zone = max(
            self._approach_mm, cfg.f('precision_zone_mm', 30.0))
        self._precision_speed = max(
            self._speed_min,
            min(self._speed_max, abs(cfg.f('precision_speed', self._speed_max))))
        self._precision_pulse = max(
            self._dt, min(0.30, cfg.f('precision_pulse_sec', 0.20)))
        self._precision_pulse_min = max(
            self._dt,
            min(self._precision_pulse, cfg.f('precision_pulse_min_sec', 0.10)))
        self._precision_full_error = max(
            self._tol * 2.0, cfg.f('precision_full_pulse_error_mm', 18.0))
        self._precision_coast = max(
            self._dt, min(1.0, cfg.f('precision_coast_sec', 0.35)))
        measured = cfg.f('ticks_per_mm', 0.0)
        if measured > 0.0:
            self._ticks_per_mm = measured
            self._tpm_source = 'measured'
        else:
            self._ticks_per_mm = ticks_per_linear_mm(
                cfg.f('encoder_cpr', 512.0), cfg.f('quadrature', 4.0),
                cfg.f('mm_per_rev', 50.0))
            self._tpm_source = 'formula'

        self._pos_mm = 0.0
        self._pos_filt = 0.0
        self._last_good_ticks: Optional[float] = None
        self._home_latch_lock = threading.Lock()
        self._home_sw = False
        self._limit_handled = False
        self._limit_release_t0: Optional[float] = None
        self._limit_reset_lock = threading.Lock()
        self._last_speed = 0.0
        self._left_home_stop_until = 0.0

        srv = cfg.s('titan_service', '/titan1/titan_cmd')
        home_topic = cfg.s('home_limit_topic', '/titan1/m_0/limit_fwd')
        self._titan = node.create_client(
            SetData, srv, callback_group=self.group)
        self._linear_pub = node.create_publisher(
            Float64, cfg.s('linear_cmd_topic', '/linear_servo/cmd'), 10)
        self._result_pub = node.create_publisher(String, '/oms/linear/result', 10)
        self._status_pub = node.create_publisher(String, '/oms/linear/status', 10)
        self._pos_pub = node.create_publisher(Float64, '/oms/linear/pos_mm', 10)
        node.create_subscription(
            Float64, cfg.s('encoder_topic', '/titan1/m_3/encoder'),
            self._on_encoder, 10, callback_group=self.group)
        node.create_subscription(
            Bool, home_topic, self._on_limit, 10, callback_group=self.group)
        node.create_subscription(
            String, '/oms/linear/cmd', self._on_cmd, 10,
            callback_group=self.group)
        node.create_subscription(
            Float64, '/oms/linear/set_pos', self._on_set_pos, 10,
            callback_group=self.group)
        node.create_timer(0.1, self.publish_status, callback_group=self.group)
        node.get_logger().info(
            f'OMS linear  Cypher M{self._enc_motor}  '
            f'ticks/mm={self._ticks_per_mm:.2f} ({self._tpm_source})  '
            f'speed={self._speed_max:.0f}  speed_sign={self._speed_sign:+.0f}  '
            f'home={home_topic}  motion_log={self._motion_log}')

    def _pos_from_ticks(self, ticks: float) -> float:
        return self._ticks_sign * (
            float(ticks) - self._home_ticks) / max(1e-6, self._ticks_per_mm)

    def _on_encoder(self, msg: Float64) -> None:
        raw = float(msg.data)
        moving = (
            self._state in ('goto', 'homing')
            or abs(self._last_speed) >= 1.0)
        # Only reject glitches while stopped — moving axis can change >80 ticks/sample.
        if (
            self._homed
            and self._last_good_ticks is not None
            and not moving
            and abs(raw - self._last_good_ticks) > self._enc_max_jump
        ):
            return
        self._ticks = raw
        self._last_good_ticks = raw
        if self._home_sw and self._homed and self._state not in ('goto', 'homing'):
            self._pos_mm = 0.0
            self._pos_filt = 0.0
            return
        if self._homed:
            pos = self._pos_from_ticks(raw)
            self._pos_filt += self._pos_alpha * (pos - self._pos_filt)
            self._pos_mm = self._pos_filt
        if self._state in ('goto', 'homing'):
            self._mlog('sample')

    def _on_limit(self, msg: Bool) -> None:
        pressed = bool(msg.data)
        rising = pressed and not self._home_sw
        falling = (not pressed) and self._home_sw
        self._home_sw = pressed
        if not pressed:
            # The switch can remain pressed while the rail builds speed. Stop
            # at its release edge so a long command does not coast tens of mm
            # before precision control gets its first useful position.
            if (
                falling
                and self._state == 'goto'
                and (self._last_speed * self._speed_sign) > 0.0
            ):
                self.stop()
                self._left_home_stop_until = time.time() + self._precision_coast
                self.log('info', 'home switch released → stop and measure coast')
            return

        self._limit_release_t0 = None
        if rising:
            # Hard stop first — continuous servo holds last cmd until we publish 0.
            self.stop()
            self.log('info', 'linear limit rising → stop + software zero (no encoder reset)')
            if not self._limit_handled:
                self._limit_handled = True
                threading.Thread(
                    target=self._limit_pressed_reset, daemon=True).start()
        elif not self._limit_handled:
            self._limit_handled = True
            threading.Thread(
                target=self._limit_pressed_reset, daemon=True).start()

    def _maybe_clear_limit_handled(self) -> None:
        if self._home_sw or not self._limit_handled:
            self._limit_release_t0 = None
            return
        now = time.time()
        if self._limit_release_t0 is None:
            self._limit_release_t0 = now
            return
        if now - self._limit_release_t0 >= 0.35:
            self._limit_handled = False
            self._limit_release_t0 = None

    def _latch_home_at_limit(self) -> None:
        """Software zero at rear limit. Never reset Cypher — home_ticks only."""
        with self._home_latch_lock:
            self.stop()
            raw = float(self._ticks) if self._ticks is not None else 0.0
            # Quick median without long wait (limit already pressed).
            samples: list[float] = []
            t0 = time.time()
            while rclpy.ok() and (time.time() - t0) < 0.12:
                if self._ticks is not None:
                    samples.append(float(self._ticks))
                time.sleep(0.02)
            if samples:
                raw = float(statistics.median(samples))
            self._home_ticks = raw
            self._last_good_ticks = raw
            self._pos_mm = 0.0
            self._pos_filt = 0.0
            self._homed = True
            if self._state not in ('goto', 'homing'):
                self._state = 'idle'
            self.log(
                'info',
                f'software home latched raw_ticks={raw:.1f} '
                f'(no Cypher reset, pos=0 mm)')
            self._mlog('homed', force=True)

    def _limit_pressed_reset(self) -> None:
        if not self._limit_reset_lock.acquire(blocking=False):
            return
        try:
            if not self._home_sw:
                return
            self.log('info', 'linear home limit → software zero only')
            self._latch_home_at_limit()
            self._mlog('limit_zero', force=True)
            self.log(
                'info',
                f'limit zero done raw={self._ticks} home_ticks={self._home_ticks:.1f}')
        finally:
            self._limit_reset_lock.release()

    def _soft_zero_at_limit(self) -> None:
        """Backward-compatible alias — always software-latch."""
        self._latch_home_at_limit()

    def _pos_now(self, *, for_control: bool = False) -> float:
        if self._home_sw and self._homed and self._state not in ('goto', 'homing'):
            return 0.0
        if self._ticks is None or not self._homed:
            return float(self._pos_mm)
        if for_control or self._state in ('goto', 'homing'):
            return self._pos_from_ticks(self._ticks)
        return float(self._pos_filt)

    def _mlog(self, event: str, *, force: bool = False) -> None:
        self._write_motion(
            event,
            f'{self._pos_now():.2f},{self._last_speed:.1f},'
            f'{int(self._home_sw)},{int(self._homed)},{self._state}',
            force)

    def publish_status(self) -> None:
        self._maybe_clear_limit_handled()
        if not self._should_publish_status():
            return
        self._pos_mm = self._pos_now()
        payload = {
            'state': self._state,
            'homed': self._homed,
            'home_switch': self._home_sw,
            'pos_mm': round(self._pos_mm, 2),
            'ticks': None if self._ticks is None else round(self._ticks, 1),
            'speed': round(self._last_speed, 1),
            'min_mm': self._min_mm,
            'max_mm': self._max_mm,
        }
        try:
            self._status_pub.publish(String(data=json.dumps(payload)))
            self._pos_pub.publish(Float64(data=float(self._pos_mm)))
        except Exception:
            pass

    def _pub_result(self, cmd_id: str, ok: bool, message: str) -> None:
        payload = {
            'cmd_id': cmd_id,
            'success': bool(ok),
            'mode': 'linear',
            'message': message,
            'pos_mm': round(self._pos_now(), 2),
            'homed': self._homed,
        }
        try:
            self._result_pub.publish(String(data=json.dumps(payload)))
        except Exception:
            pass
        self.log('info' if ok else 'warn', f'result ok={ok} {message}')

    def _titan_reset(self) -> bool:
        """Optional diagnostic only — limit/home path never calls this."""
        if not self._titan.service_is_ready():
            self.log('warn', 'reset_encoder: titan not ready')
            return False
        ok = self._titan_call(
            self._titan, self._enc_motor, 'reset_encoder', 0.0, 2.0)
        time.sleep(0.08)
        self.log(
            'info',
            f'reset_encoder sent ok={ok} raw_ticks={self._ticks} '
            '(not used on limit — software home_ticks is authoritative)')
        return ok

    def _apply_speed(self, speed: float) -> None:
        speed = max(-100.0, min(100.0, float(speed)))
        # On rear limit: never command rearward. Forward leave-home only.
        if self._home_sw and (speed * self._speed_sign) < 0.0:
            speed = 0.0
        if abs(speed) < 1e-9:
            self.stop()
            return
        step = max(0.5, self._duty_step * 100.0)
        if abs(speed - self._last_speed) < step:
            return
        self._last_speed = speed
        self._linear_pub.publish(Float64(data=speed))

    def stop(self) -> None:
        self._last_speed = 0.0
        # Burst zeros so continuous Smart Servo 200 actually coasts to stop.
        for _ in range(3):
            try:
                self._linear_pub.publish(Float64(data=0.0))
            except Exception:
                break

    def emergency_stop(self) -> None:
        try:
            self.stop()
        except Exception:
            pass

    def _mark_home(self) -> None:
        self._latch_home_at_limit()
        self._state = 'idle'

    def _fwd_speed(self, mag: float) -> float:
        """Command that increases pos_mm (leave rear home)."""
        return self._speed_sign * abs(mag)

    def _rear_speed(self, mag: float) -> float:
        """Command that decreases pos_mm / drives onto rear limit."""
        return -self._speed_sign * abs(mag)

    def _drive_until_home(self, speed: float, timeout: float, gen: int) -> str:
        """Drive toward rear limit; creep near zero but never below breakaway speed."""
        approach_mm = max(self._tol * 3.0, float(self.cfg.f('home_approach_mm', 20.0)))
        # Continuous Smart Servo needs ~10+ or it won't overcome stiction.
        breakaway = max(10.0, self._speed_min)
        creep = max(breakaway, min(abs(speed), self._home_speed))
        t0 = time.time()
        last_move = time.time()
        last_ticks = self._ticks
        stall_arm = t0 + 2.0
        while self._alive(gen):
            if self._home_sw:
                self.stop()
                return 'home'
            cmd = speed
            if self._ticks is not None:
                pos = self._pos_from_ticks(float(self._ticks))
                if 0.0 <= pos <= approach_mm:
                    cmd = self._rear_speed(creep)
                elif pos < 0.0:
                    # Past software zero, still no switch — keep creeping onto switch.
                    cmd = self._rear_speed(creep)
            self._apply_speed(cmd)
            now = time.time()
            if now - t0 >= timeout:
                self.stop()
                return 'timeout'
            ticks = self._ticks
            if ticks is not None and last_ticks is not None:
                if abs(ticks - last_ticks) >= self._stall_ticks:
                    last_move = now
                    last_ticks = ticks
                elif (
                    now >= stall_arm
                    and abs(cmd) >= breakaway
                    and (now - last_move) >= max(1.5, self._stall_sec)
                ):
                    self.stop()
                    return 'stall'
            time.sleep(self._dt)
        self.stop()
        return 'cancel'

    def _search_home(self, gen: int) -> bool:
        self._state = 'homing'
        # Keep prior home_ticks so pos_from_ticks still works for approach creep.
        if self._home_sw:
            self.stop()
            self._mark_home()
            return True
        self._cmd_text = 'home'
        self._cmd_target = 0.0
        rear = self._rear_speed(self._home_speed)
        self.log(
            'info',
            f'home rearward cmd={rear:+.0f}  speed_sign={self._speed_sign:+.0f}  '
            f'home_sw={self._home_sw} ticks={self._ticks} '
            f'approach≤{self.cfg.f("home_approach_mm", 20.0):.0f}mm')
        outcome = self._drive_until_home(rear, self._home_timeout, gen)
        if outcome == 'home':
            self._mark_home()
            return True
        self.stop()
        self._state = 'home_failed'
        self.log('error', f'home failed ({outcome})')
        return False

    def _goto(self, target: float, gen: int) -> str:
        target = max(self._min_mm, min(self._max_mm, float(target)))
        if not self._homed and not self._search_home(gen):
            return 'home failed'
        self._state = 'goto'
        self._pid.reset()
        if self._ticks is not None:
            self._last_good_ticks = float(self._ticks)
            self._pos_filt = self._pos_from_ticks(self._ticks)
            self._pos_mm = self._pos_filt
        start_pos = self._pos_now(for_control=True)
        start_ticks = self._ticks
        t0 = time.time()
        in_tol_t: Optional[float] = None
        last = time.time()
        ticks_warn = False
        last_tick_move = time.time()
        last_tick_val = start_ticks
        precision_settled = False
        # Only treat as encoder-dead if commanding hard enough to move the rail.
        breakaway = max(10.0, self._speed_min)
        enc_stall_abort = max(3.5, self._stall_sec * 4.0)
        self._cmd_text = f'goto {target:.1f}'
        self._cmd_target = target
        self.log(
            'info',
            f'goto {target:.1f} mm from {start_pos:.1f} mm  '
            f'home_sw={int(self._home_sw)} ticks={self._ticks} '
            f'home_ticks={self._home_ticks:.1f} speed_sign={self._speed_sign:+.0f}')
        self._mlog('goto_start', force=True)
        while self._alive(gen):
            now = time.time()
            if now - t0 >= self._goto_timeout:
                self.stop()
                self._state = 'idle'
                self.log('error', f'goto timeout pos={self._pos_now(for_control=True):.1f} mm')
                return 'timeout'
            if now < self._left_home_stop_until:
                self.stop()
                time.sleep(self._dt)
                continue
            if self._home_sw and target <= self._min_mm + self._tol:
                self._mark_home()
                return 'ok'
            pos = self._pos_now(for_control=True)
            err = target - pos
            dt = max(self._dt, now - last)
            last = now
            ticks = self._ticks
            if (
                ticks is not None
                and last_tick_val is not None
                and abs(float(ticks) - float(last_tick_val)) >= self._stall_ticks
            ):
                last_tick_move = now
                last_tick_val = ticks
            elif ticks is not None and last_tick_val is None:
                last_tick_val = ticks
                last_tick_move = now
            if (
                not ticks_warn
                and now - t0 >= 1.5
                and abs(self._last_speed) >= breakaway
                and ticks is not None
                and start_ticks is not None
                and abs(float(ticks) - float(start_ticks)) < self._stall_ticks
            ):
                ticks_warn = True
                self.log(
                    'warn',
                    f'encoder M3 not changing (ticks={ticks}) while '
                    f'cmd={self._last_speed:+.1f} — check Cypher on Titan1 M3 / '
                    f'/titan1/m_3/encoder; if motion is wrong way flip speed_sign')
            # Stall abort only when cmd is above breakaway (low cmd ≠ dead encoder).
            if (
                abs(self._last_speed) >= breakaway
                and not self._home_sw
                and ticks is not None
                and (now - last_tick_move) >= enc_stall_abort
                and (now - t0) >= enc_stall_abort
            ):
                self.stop()
                self._state = 'idle'
                self.log(
                    'error',
                    f'goto aborted — encoder stall ticks={ticks} '
                    f'pos={pos:.1f} mm (wanted {target:.1f} mm). '
                    f'Fix Cypher M3 wiring/topic before closed-loop moves.')
                self._mlog('enc_stall', force=True)
                return 'encoder stall'
            if abs(err) <= self._tol:
                self.stop()
                if in_tol_t is None:
                    in_tol_t = now
                elif now - in_tol_t >= self._settle:
                    self._state = 'idle'
                    self._mlog('arrived', force=True)
                    self.log(
                        'info',
                        f'arrived target={target:.1f} mm pos={pos:.1f} mm '
                        f'ticks={self._ticks}')
                    return 'ok'
            else:
                in_tol_t = None
                if self._home_sw and err < 0:
                    self.stop()
                    self._mark_home()
                    return 'ok'
                # Leave the physical switch at minimum usable speed.  Using
                # cruise speed here caused a 40 mm command to coast past 50 mm
                # before the first useful encoder correction.
                if self._home_sw and err > self._tol:
                    leave = self._fwd_speed(self._speed_min)
                    if abs(self._last_speed) < 1.0:
                        self.log(
                            'info',
                            f'leave-home cmd={leave:+.0f} '
                            f'(pos={pos:.1f} home_sw={int(self._home_sw)})')
                    self._apply_speed(leave)
                elif abs(err) <= self._precision_zone:
                    # On first entering precision control, stop the previous
                    # sustained move and measure its coast before adding a pulse.
                    if not precision_settled and abs(self._last_speed) >= 1.0:
                        self.stop()
                        if not self._sleep(self._precision_coast, self._dt):
                            return 'cancel'
                        precision_settled = True
                        continue
                    precision_settled = True
                    # Never ask for a sustained command below breakaway.  Pulse
                    # at loaded precision speed, stop, then let the encoder report the
                    # coasted position before deciding the next direction.
                    pulse = (
                        self._fwd_speed(self._precision_speed)
                        if err > 0.0 else self._rear_speed(self._precision_speed))
                    pulse_sec = max(
                        self._precision_pulse_min,
                        self._precision_pulse * min(
                            1.0, abs(err) / self._precision_full_error))
                    self._apply_speed(pulse)
                    # Refresh throughout the pulse; one 80 ms command only
                    # twitched the motor and did not move the loaded rack.
                    pulse_until = time.time() + pulse_sec
                    while self._alive(gen) and time.time() < pulse_until:
                        self._linear_pub.publish(Float64(data=pulse))
                        self._last_speed = pulse
                        time.sleep(self._dt)
                    if not self._alive(gen):
                        self.stop()
                        return 'cancel'
                    self.stop()
                    if not self._sleep(self._precision_coast, self._dt):
                        return 'cancel'
                else:
                    precision_settled = False
                    raw = self._speed_sign * self._pid.update(err, dt)
                    if abs(err) <= self._approach_mm:
                        cap = self._speed_max * min(
                            1.0, abs(err) / max(self._tol, self._approach_mm))
                        # Near target: allow below cruise breakaway so we don't hunt ±12.
                        near_floor = max(6.0, self._speed_min * 0.65)
                        speed = max(-cap, min(cap, raw * self._speed_max))
                        if abs(err) <= self._tol * 1.5 and abs(speed) < 6.0:
                            speed = 0.0
                        elif abs(speed) > 1e-9 and abs(speed) < near_floor:
                            speed = math.copysign(near_floor, speed)
                    else:
                        speed = _shape(raw, breakaway, self._speed_max)
                    self._apply_speed(speed)
            time.sleep(self._dt)
        self.stop()
        return 'cancel'

    def wait_ready(self) -> bool:
        if not self._wait_titan(self._titan):
            self._state = 'home_failed'
            self.log('error', 'titan not ready')
            return False
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < 10.0:
            if self._ticks is not None:
                break
            time.sleep(0.05)
        return True

    def home_blocking(self, cmd_id: str) -> bool:
        gen = self._begin_job()
        ok = self._search_home(gen)
        self._pub_result(cmd_id, ok, 'homed' if ok else 'home failed')
        return ok

    def _on_set_pos(self, msg: Float64) -> None:
        self._on_cmd(String(data=json.dumps({
            'cmd_id': f'set_{int(time.time() * 1000)}',
            'mode': 'goto',
            'pos_mm': float(msg.data),
        })))

    def _on_cmd(self, msg: String) -> None:
        try:
            cmd = json.loads(msg.data)
        except Exception as exc:
            self._pub_result('', False, f'bad JSON: {exc}')
            return
        if not isinstance(cmd, dict):
            self._pub_result('', False, 'command must be an object')
            return
        mode = str(cmd.get('mode') or 'goto').strip().lower()
        self._cmd_text = mode
        threading.Thread(target=self._run_cmd, args=(cmd, mode), daemon=True).start()

    def _run_cmd(self, cmd: dict, mode: str) -> None:
        cmd_id = str(cmd.get('cmd_id') or '')
        if mode in ('stop', 'cancel'):
            self.cancel()
            self.stop()
            self._state = 'idle'
            self._pub_result(cmd_id, True, 'stopped')
            return
        if mode in ('home', 'back'):
            self.home_blocking(cmd_id)
            return
        if mode == 'goto':
            try:
                target = float(cmd.get('pos_mm'))
            except (TypeError, ValueError):
                self._pub_result(cmd_id, False, 'missing pos_mm')
                return
            gen = self._begin_job()
            outcome = self._goto(target, gen)
            self._pub_result(cmd_id, outcome == 'ok', outcome)
            return
        self._pub_result(cmd_id, False, f'unknown mode {mode}')


class ServoAxis(_Axis):
    """Standard Studica servo: two poses, home_deg / alt_deg in oms.yaml.

    Used by rotation_servo and gripper. Vertical overrides the key and pose
    names below because its yaml block and status payload use up/down.
    """

    home_key = 'home_deg'
    alt_key = 'alt_deg'
    home_label = 'home'

    def __init__(self, node: 'OmsNode', cfg: _Cfg, topic: str, label: str,
                 default_service: str) -> None:
        super().__init__(node, cfg)
        self.label = label
        self._topic = topic
        self._home = cfg.f(self.home_key, 0.0)
        self._alt = cfg.f(self.alt_key, 0.0)
        self._alt_mode = cfg.s('alt_mode', 'alt').strip().lower()
        self._settle = max(0.2, cfg.f('settle_sec', 0.8))
        # Standard PWM servos jump instantly; optional software ramp (°/s).
        self._slew_dps = max(0.0, cfg.f('slew_deg_per_sec', 0.0))
        self._slew_hz = max(5.0, cfg.f('slew_hz', 20.0))
        self._pose = 'unknown'
        self._angle: Optional[float] = None
        self._status_last = 0.0
        self._status_hz_active = max(2.0, cfg.f('status_hz_active', 5.0))
        self._status_hz_idle = max(0.5, cfg.f('status_hz_idle', 2.0))

        srv = cfg.s('service', default_service)
        # Studica PWM path: /<name>/cmd Float64 → SetAngle → PWMGenerator_SetDutyCycle.
        # Prefer this over SetData for ramps (no service round-trip / timing hacks).
        hw_cmd = cfg.s('hw_cmd_topic', '')
        if not hw_cmd:
            # /rotation_servo/set_servo → /rotation_servo/cmd
            if srv.endswith('/set_servo'):
                hw_cmd = srv[: -len('/set_servo')] + '/cmd'
            else:
                hw_cmd = f'/{topic}/cmd'
        self._cli = node.create_client(SetData, srv, callback_group=self.group)
        self._hw_cmd_pub = node.create_publisher(Float64, hw_cmd, 10)
        self._hw_cmd_topic = hw_cmd
        self._result_pub = node.create_publisher(
            String, f'/oms/{topic}/result', 10)
        self._status_pub = node.create_publisher(
            String, f'/oms/{topic}/status', 10)
        node.create_subscription(
            String, f'/oms/{topic}/cmd', self._on_cmd, 10,
            callback_group=self.group)
        node.create_subscription(
            String, f'/oms/{topic}/set_pose', self._on_set_pose, 10,
            callback_group=self.group)
        node.create_timer(0.2, self.publish_status, callback_group=self.group)
        node.get_logger().info(self.banner(srv))

    def banner(self, srv: str) -> str:
        slew = (
            f'  slew={self._slew_dps:g}°/s'
            if self._slew_dps > 0.0 else '  slew=off')
        return (f'OMS {self.label}  {srv}  pwm={self._hw_cmd_topic}  '
                f'home={self._home:g}°  {self._alt_mode}={self._alt:g}°{slew}')

    def publish_status(self) -> None:
        moving = self._state not in ('idle', 'boot', 'unknown')
        hz = self._status_hz_active if moving else self._status_hz_idle
        now = time.time()
        if now - self._status_last < (1.0 / hz):
            return
        self._status_last = now
        payload = {
            'state': self._state,
            'homed': self._homed,
            'pose': self._pose,
            'angle_deg': self._angle,
            self.home_key: self._home,
            self.alt_key: self._alt,
        }
        try:
            self._status_pub.publish(String(data=json.dumps(payload)))
        except Exception:
            pass

    def _pub_result(self, cmd_id: str, ok: bool, message: str) -> None:
        payload = {
            'cmd_id': cmd_id,
            'success': bool(ok),
            'mode': self._topic,
            'message': message,
            'pose': self._pose,
            'angle_deg': self._angle,
            'homed': self._homed,
        }
        try:
            self._result_pub.publish(String(data=json.dumps(payload)))
        except Exception:
            pass
        self.log('info' if ok else 'warn', f'result ok={ok} {message}')

    def wait_ready(self, timeout: float = 20.0) -> bool:
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < timeout:
            if self._cli.wait_for_service(timeout_sec=0.5):
                return True
        return self._cli.service_is_ready()

    def _set_angle(self, deg: float, *, confirm: bool = False) -> bool:
        """Write PWM duty via Studica /cmd (angle → SetAngle → SetDutyCycle).

        Standard servos have no hardware velocity — slow motion = stepped angles.
        ``confirm=True`` also hits set_servo once (final latch / readiness check).
        """
        deg_i = float(int(round(float(deg))))
        try:
            self._hw_cmd_pub.publish(Float64(data=deg_i))
        except Exception:
            pass
        if not confirm:
            return True
        if not self._cli.service_is_ready():
            return False
        req = SetData.Request()
        req.initparams.speed = deg_i
        fut = self._cli.call_async(req)
        t0 = time.time()
        while rclpy.ok() and not fut.done() and (time.time() - t0) < 8.0:
            time.sleep(0.02)
        if not fut.done():
            return False
        try:
            res = fut.result()
        except Exception:
            return False
        if bool(getattr(res, 'success', False)):
            return True
        msg = str(getattr(res, 'message', '') or '').lower()
        return 'already' in msg

    def _resolve_mode(self, mode: str) -> tuple[str, float]:
        mode = (mode or self.home_label).strip().lower()
        if mode in {'home', 'down', 'close', 'out', 'o'}:
            return self.home_label, self._home
        if mode in (self._alt_mode, 'up', 'open', 'in', 'i', 'alt'):
            return self._alt_mode, self._alt
        if mode == 'goto':
            return 'goto', self._home
        return mode, self._home

    def _pose_of(self, target: float) -> str:
        return (self.home_label if abs(target - self._home) < 0.5
                else self._alt_mode)

    def _goto_result(self, target: float) -> str:
        return 'ok'

    def _goto_deg(self, target: float) -> bool:
        """Drive standard servo by stepping PWM duty (degrees).

        Hardware has no velocity input — only PWM angle. Slow motion = publish
        successive angles on /<servo>/cmd (SetAngle → SetDutyCycle).
        """
        self._state = 'goto'
        target = float(int(round(float(target))))
        start = float(self._angle) if self._angle is not None else float(target)
        start = float(int(round(start)))
        ok = False
        used_slew = False
        if self._slew_dps <= 0.0 or abs(target - start) < 1.0:
            for attempt in range(1, 6):
                ok = self._set_angle(target, confirm=True)
                if ok:
                    break
                self.log('warn', f'attempt {attempt}/5 failed — retry')
                time.sleep(0.25)
        else:
            # Integer-degree PWM ramp — Studica SetAngle uses int degrees.
            used_slew = True
            dt = 1.0 / self._slew_hz
            step = max(1.0, round(self._slew_dps * dt))
            cur = start
            direction = 1.0 if target >= start else -1.0
            self.log(
                'info',
                f'pwm ramp {start:g}° → {target:g}° at ~{self._slew_dps:g}°/s '
                f'(step={step:g}° via {self._hw_cmd_topic})')
            t_next = time.time()
            while abs(target - cur) > step:
                cur = cur + direction * step
                if (direction > 0 and cur > target) or (
                        direction < 0 and cur < target):
                    cur = target
                self._set_angle(cur, confirm=False)
                self._angle = cur
                t_next += dt
                sleep_for = t_next - time.time()
                if sleep_for > 0.0:
                    time.sleep(sleep_for)
            ok = self._set_angle(target, confirm=True)
            if not ok:
                # Topic already updated; service confirm optional.
                self._set_angle(target, confirm=False)
                ok = True
        if ok:
            # After a PWM ramp the horn is already at target — short settle only.
            settle = min(self._settle, 0.25) if used_slew else self._settle
            if settle > 0.0:
                time.sleep(settle)
            self._angle = target
            self._pose = self._pose_of(target)
            self._homed = abs(target - self._home) < 0.5
            self._state = 'idle'
            self.log('info', f'→ {target:g}°')
        else:
            self._state = 'failed'
            self.log('error', f'servo move failed ({self.label})')
        return ok

    def home_blocking(self, cmd_id: str) -> bool:
        self._state = 'wait_servo'
        if not self.wait_ready():
            self._state = 'home_failed'
            self.log('error', 'servo service not ready')
            self._pub_result(cmd_id, False, 'home failed')
            return False
        ok = self._goto_deg(self._home)
        if ok:
            self._homed = True
            self._pose = self.home_label
        self._pub_result(cmd_id, ok, self.home_label if ok else 'home failed')
        return ok

    def _on_set_pose(self, msg: String) -> None:
        self._on_cmd(String(data=json.dumps({
            'cmd_id': f'set_{int(time.time() * 1000)}',
            'mode': str(msg.data or 'home').strip().lower(),
        })))

    def _on_cmd(self, msg: String) -> None:
        try:
            cmd = json.loads(msg.data)
        except Exception as exc:
            self._pub_result('', False, f'bad JSON: {exc}')
            return
        if not isinstance(cmd, dict):
            self._pub_result('', False, 'command must be an object')
            return
        threading.Thread(target=self._run_cmd, args=(cmd,), daemon=True).start()

    def _run_cmd(self, cmd: dict) -> None:
        self._begin_job()
        cmd_id = str(cmd.get('cmd_id') or '')
        mode = str(cmd.get('mode') or self.home_label).strip().lower()
        for key, attr in ((self.home_key, '_home'), (self.alt_key, '_alt')):
            if cmd.get(key) is not None:
                try:
                    setattr(self, attr, float(cmd[key]))
                except (TypeError, ValueError):
                    pass
        if mode in ('stop', 'cancel'):
            self._state = 'idle'
            self._pub_result(cmd_id, True, 'stopped')
            return
        if mode == 'goto':
            try:
                target = float(cmd['angle_deg'])
            except (KeyError, TypeError, ValueError):
                self._pub_result(cmd_id, False, 'missing angle_deg')
                return
            ok = self._goto_deg(target)
            self._pub_result(
                cmd_id, ok, self._goto_result(target) if ok else 'goto failed')
            return
        label, target = self._resolve_mode(mode)
        ok = self._goto_deg(target)
        self._pub_result(cmd_id, ok, label if ok else f'{label} failed')


class RotationServoAxis(ServoAxis):
    """Rotation wrist: home = in (yaml home_deg), alt = out (yaml alt_deg)."""

    home_label = 'in'

    def _resolve_mode(self, mode: str) -> tuple[str, float]:
        mode = (mode or self.home_label).strip().lower()
        # Primary names: in / out. Keep old horiz/vert aliases for any stale clients.
        if mode in ('in', 'i', 'horizontal', 'horiz', 'h', 'home'):
            return 'in', self._home
        if mode in ('out', 'o', 'vertical', 'vert', 'v'):
            return 'out', self._alt
        if mode == 'goto':
            return 'goto', self._home
        return mode, self._home

    def _pose_of(self, target: float) -> str:
        return ('in' if abs(target - self._home) < 0.5
                else 'out')


class VerticalServoAxis(ServoAxis):
    """Vertical servo: oms.yaml uses up_deg / down_deg, poses are up / down."""

    home_key = 'down_deg'
    alt_key = 'up_deg'
    home_label = 'down'

    def __init__(self, node: 'OmsNode', cfg: _Cfg) -> None:
        super().__init__(
            node, cfg, 'vertical', 'oms_vertical', '/vertical_servo/set_servo')
        self._alt_mode = 'up'

    def banner(self, srv: str) -> str:
        return (f'OMS vertical  {srv}  up={self._alt:g}°  '
                f'down(home)={self._home:g}°')

    def _pose_of(self, target: float) -> str:
        # Old node labelled an arbitrary goto by whichever pose is nearer.
        return ('up' if abs(target - self._alt) <= abs(target - self._home)
                else 'down')

    def _goto_result(self, target: float) -> str:
        return self._pose_of(target)


class HomeSequencer:
    """Boot / on-demand homing across the axes, in an editable order."""

    def __init__(self, node: 'OmsNode', cfg: _Cfg, axes: dict) -> None:
        self.node = node
        self._axes = axes
        self._step_timeout = max(5.0, cfg.f('step_timeout_sec', 60.0))
        self._inter = max(0.0, cfg.f('inter_step_sec', 0.5))
        self._state = 'idle'
        self._message = ''
        self._current = ''
        self._failed = ''
        self._active_seq = self._parse(cfg.raw('sequence'))
        self._steps = self._blank(self._active_seq)
        self._busy = threading.Lock()
        self._status_last = 0.0
        self.group = MutuallyExclusiveCallbackGroup()
        self._status_pub = node.create_publisher(String, '/oms/home/status', 10)
        self._result_pub = node.create_publisher(String, '/oms/home/result', 10)
        node.create_subscription(
            String, '/oms/home/cmd', self._on_cmd, 10,
            callback_group=self.group)
        node.create_timer(0.25, self.publish_status, callback_group=self.group)
        node.get_logger().info(
            'OMS home sequence: ' + ' → '.join(self._active_seq))

    def _parse(self, raw) -> list:
        if isinstance(raw, str):
            raw = [p.strip() for p in raw.replace('→', ',').split(',')]
        names = []
        for item in list(raw or []):
            name = str(item).strip().lower()
            if name in self._axes and name not in names:
                names.append(name)
        return names or [n for n in HOME_STEPS if n in self._axes]

    def _blank(self, seq: list) -> dict:
        return {n: {'status': 'pending', 'message': ''} for n in seq}

    def publish_status(self, *, force: bool = False) -> None:
        if not force and self._state == 'idle':
            now = time.time()
            if now - self._status_last < 0.5:
                return
            self._status_last = now
        try:
            self._status_pub.publish(String(data=json.dumps({
                'state': self._state,
                'sequence': list(self._active_seq),
                'current': self._current,
                'failed_step': self._failed,
                'steps': self._steps,
                'message': self._message,
            })))
        except Exception:
            pass

    def _mark(self, name: str, status: str, message: str = '') -> None:
        slot = self._steps.setdefault(name, {})
        slot['status'] = status
        slot['message'] = message
        self.publish_status(force=True)

    def _pub_result(self, ok: bool, message: str) -> None:
        try:
            self._result_pub.publish(String(data=json.dumps({
                'cmd_id': '',
                'success': bool(ok),
                'mode': 'home',
                'message': message,
                'failed_step': self._failed,
                'steps': self._steps,
                'sequence': list(self._active_seq),
            })))
        except Exception:
            pass

    def _run_step(self, name: str) -> tuple[bool, str]:
        """Home one axis with the sequence-level timeout as a safety net."""
        axis = self._axes[name]
        cmd_id = f'home_{name}_{int(time.time() * 1000)}'
        outcome = {'ok': False, 'msg': ''}

        def work() -> None:
            try:
                outcome['ok'] = bool(axis.home_blocking(cmd_id))
                outcome['msg'] = 'homed' if outcome['ok'] else 'failed'
            except Exception as exc:  # keep the sequence alive
                outcome['ok'] = False
                outcome['msg'] = f'error: {exc}'

        worker = threading.Thread(target=work, daemon=True)
        self.node.get_logger().info(f'[{name}] home → {cmd_id}')
        worker.start()
        worker.join(self._step_timeout)
        if worker.is_alive():
            axis.cancel()
            axis.emergency_stop()
            msg = f'timeout {self._step_timeout:.0f}s'
            self.node.get_logger().error(f'[{name}] {msg}')
            return False, msg
        self.node.get_logger().info(
            f'[{name}] ok={outcome["ok"]} {outcome["msg"]}')
        return bool(outcome['ok']), str(outcome['msg'])

    def run(self, sequence=None) -> bool:
        if not self._busy.acquire(blocking=False):
            self.node.get_logger().warn('home sequence already running')
            return False
        try:
            seq = self._parse(sequence) if sequence else list(self._active_seq)
            self._active_seq = seq
            self._steps = self._blank(seq)
            self._failed = ''
            self._current = seq[0] if seq else ''
            self._state = 'homing'
            self._message = ' → '.join(seq)
            self.publish_status()
            for index, name in enumerate(seq):
                self._current = name
                self._mark(name, 'running')
                ok, step_msg = self._run_step(name)
                if not ok:
                    self._failed = name
                    self._state = 'home_failed'
                    self._message = f'{name}: {step_msg}'
                    self._mark(name, 'fail', step_msg)
                    for rest in seq[index + 1:]:
                        self._mark(rest, 'skip', 'not run')
                    self._pub_result(False, f'home failed at {name}')
                    return False
                self._mark(name, 'ok', step_msg or 'homed')
                time.sleep(self._inter)
            self._current = ''
            self._state = 'idle'
            self._message = 'all homed'
            self.node.get_logger().info('OMS home sequence complete')
            self.publish_status()
            self._pub_result(True, 'all homed')
            return True
        finally:
            self._busy.release()

    def _on_cmd(self, msg: String) -> None:
        try:
            cmd = json.loads(msg.data)
        except Exception:
            return
        if not isinstance(cmd, dict):
            return
        if str(cmd.get('mode') or '').strip().lower() not in (
                'home', 'allhome', 'boot'):
            return
        threading.Thread(
            target=self.run, args=(cmd.get('sequence'),), daemon=True).start()


class OmsNode(Node):
    """The single OMS node: every mechanism plus the homing sequence."""

    def __init__(self) -> None:
        super().__init__('oms')
        self._log_path = _studio_log_path('oms.log')
        config_file = declare_str(self, 'config_file', '')
        rot_log = declare_str(self, 'motion_log', '')
        lift_log = declare_str(self, 'lift_motion_log', '')
        linear_log = declare_str(self, 'linear_motion_log', '')
        # Launch / Studio param wins; yaml oms_home.auto_home is the fallback.
        # Default false — mechanisms stay idle until the dashboard Home button.
        auto_home_override = declare_bool(self, 'auto_home', False)

        data = self._load_config(config_file)
        self.rotation = RotationAxis(self, _Cfg(data, 'oms_rotation'), rot_log)
        self.lift = LiftAxis(self, _Cfg(data, 'oms_lift'), lift_log)
        self.linear = LinearAxis(self, _Cfg(data, 'oms_linear'), linear_log)
        self.vertical = VerticalServoAxis(self, _Cfg(data, 'oms_vertical'))
        self.rotation_servo = RotationServoAxis(
            self, _Cfg(data, 'oms_rotation_servo'), 'rotation_servo',
            'oms_rotation_servo', '/rotation_servo/set_servo')
        self.gripper = ServoAxis(
            self, _Cfg(data, 'oms_gripper'), 'gripper', 'oms_gripper',
            '/gripper_servo/set_servo')
        self.axes = {
            'rotation': self.rotation,
            'lift': self.lift,
            'linear': self.linear,
            'vertical': self.vertical,
            'rotation_servo': self.rotation_servo,
            'gripper': self.gripper,
        }
        home_cfg = _Cfg(data, 'oms_home')
        self.home = HomeSequencer(self, home_cfg, self.axes)
        self.get_logger().info(
            'OMS single node ready — rotation, lift, linear, vertical, '
            'rotation_servo, gripper, home (idle until Home)')
        do_auto = bool(auto_home_override) and home_cfg.b('auto_home', False)
        if do_auto:
            delay = max(0.0, home_cfg.f('startup_delay_sec', 2.0))
            threading.Thread(target=self._boot, args=(delay,), daemon=True).start()
        else:
            self.write_log(
                'info',
                'auto_home off — waiting for /oms/home/cmd (Studio Home)')

    def _load_config(self, config_file: str) -> dict:
        path = Path(config_file).expanduser() if config_file else None
        try:
            data = load_oms_yaml(path)
            self.get_logger().info(
                f'OMS config: {path or "oms.yaml (workspace/install)"}')
            return data if isinstance(data, dict) else {}
        except Exception as exc:
            self.get_logger().error(
                f'cannot read oms.yaml ({exc}) — using built-in defaults')
            return {}

    def write_log(self, level: str, msg: str) -> None:
        log = self.get_logger()
        if level == 'error':
            log.error(msg)
        elif level == 'warn':
            log.warn(msg)
        else:
            log.info(msg)
        try:
            self._log_path.parent.mkdir(parents=True, exist_ok=True)
            with open(self._log_path, 'a', encoding='utf-8') as handle:
                handle.write(f'{time.strftime("%H:%M:%S")} [{level}] {msg}\n')
        except Exception:
            pass

    def _boot(self, delay: float) -> None:
        time.sleep(delay)
        for axis in (self.rotation, self.lift, self.linear):
            if not axis.wait_ready():
                self.get_logger().error(
                    'OMS boot: titan1 not ready — home sequence skipped')
                return
        time.sleep(0.4)
        if not self.home.run():
            self.get_logger().error('OMS home sequence FAILED')

    def stop_all(self) -> None:
        for axis in self.axes.values():
            axis.cancel()
            axis.emergency_stop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OmsNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop_all()
        except Exception:
            pass
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
