#!/usr/bin/env python3
"""OMS rotation on Titan1 M2.

Home: Titan1 M0 lim_rev = 0° (front). Range +90° ACW / −90° CW (ROS).
Duty default 0.1. Negative duty = ACW = +deg (same as oms_node Q key).

On power-up: auto-home. If the switch is not pressed, search one way;
stall at a mechanical stop → reverse until the switch clicks.

Goto uses encoder PID. If duty is applied and ticks do not move, treat as
a mechanical stop: zero velocity, reverse to the home switch, mark 0°.
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String
from studica_control.srv import SetData

from oms.kinematics import ticks_per_oms_deg


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


def _shape_duty(raw: float, umin: float, umax: float) -> float:
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


class OmsRotationNode(Node):
    def __init__(self) -> None:
        super().__init__('oms_rotation')
        self.declare_parameter('titan_service', '/titan1/titan_cmd')
        self.declare_parameter('rotate_motor', 2)
        self.declare_parameter('encoder_topic', '/titan1/m_2/encoder')
        self.declare_parameter('cmd_topic', '/titan1/m_2/cmd')
        self.declare_parameter('home_limit_topic', '/titan1/m_0/limit_rev')
        self.declare_parameter('duty', 0.10)
        self.declare_parameter('duty_min', 0.06)
        self.declare_parameter('home_duty', 0.20)
        self.declare_parameter('duty_sign', -1.0)
        self.declare_parameter('ticks_sign', 1.0)
        self.declare_parameter('min_deg', -90.0)
        self.declare_parameter('max_deg', 90.0)
        self.declare_parameter('angle_tol_deg', 1.5)
        self.declare_parameter('settle_sec', 0.25)
        self.declare_parameter('goto_timeout_sec', 20.0)
        self.declare_parameter('home_timeout_sec', 25.0)
        self.declare_parameter('stall_sec', 0.70)
        self.declare_parameter('stall_ticks', 6.0)
        self.declare_parameter('stall_travel_deg', 25.0)
        self.declare_parameter('control_hz', 20.0)
        self.declare_parameter('auto_home', True)
        self.declare_parameter('home_search_sign', 1.0)
        self.declare_parameter('kp', 0.08)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('i_limit', 0.2)
        self.declare_parameter('encoder_cpr', 6.0)
        self.declare_parameter('quadrature', 4.0)
        self.declare_parameter('gearbox_ratio', 71.2)
        self.declare_parameter('sun_teeth', 120.0)
        self.declare_parameter('pinion_teeth', 32.0)
        # If > 0, use measured ticks/deg instead of the formula.
        self.declare_parameter('ticks_per_deg', 0.0)
        self.declare_parameter(
            'motion_log',
            str(_studio_log_path('oms_motion.csv')))
        self.declare_parameter('motion_log_hz', 10.0)

        self._motor = int(self.get_parameter('rotate_motor').value)
        duty = abs(float(self.get_parameter('duty').value))
        self._duty_max = max(0.02, min(1.0, duty))
        dmin = abs(float(self.get_parameter('duty_min').value))
        self._duty_min = max(0.02, min(self._duty_max, dmin))
        hd = abs(float(self.get_parameter('home_duty').value))
        self._home_duty = max(self._duty_max, min(1.0, hd))
        dsign = float(self.get_parameter('duty_sign').value)
        self._duty_sign = -1.0 if dsign < 0.0 else 1.0
        tsign = float(self.get_parameter('ticks_sign').value)
        self._ticks_sign = -1.0 if tsign < 0.0 else 1.0
        self._min_deg = float(self.get_parameter('min_deg').value)
        self._max_deg = float(self.get_parameter('max_deg').value)
        if self._min_deg > self._max_deg:
            self._min_deg, self._max_deg = self._max_deg, self._min_deg
        self._tol = max(0.2, float(self.get_parameter('angle_tol_deg').value))
        self._settle = max(0.05, float(self.get_parameter('settle_sec').value))
        self._goto_timeout = max(4.0, float(self.get_parameter('goto_timeout_sec').value))
        self._home_timeout = max(4.0, float(self.get_parameter('home_timeout_sec').value))
        self._stall_sec = max(0.2, float(self.get_parameter('stall_sec').value))
        self._stall_ticks = max(1.0, float(self.get_parameter('stall_ticks').value))
        self._stall_travel = max(5.0, float(self.get_parameter('stall_travel_deg').value))
        self._dt = 1.0 / max(5.0, float(self.get_parameter('control_hz').value))
        hsign = float(self.get_parameter('home_search_sign').value)
        self._home_search_sign = -1.0 if hsign < 0.0 else 1.0
        measured_tpd = float(self.get_parameter('ticks_per_deg').value)
        if measured_tpd > 0.0:
            self._ticks_per_deg = measured_tpd
            self._tpd_source = 'measured'
        else:
            self._ticks_per_deg = ticks_per_oms_deg(
                float(self.get_parameter('encoder_cpr').value),
                float(self.get_parameter('quadrature').value),
                float(self.get_parameter('gearbox_ratio').value),
                float(self.get_parameter('sun_teeth').value),
                float(self.get_parameter('pinion_teeth').value),
            )
            self._tpd_source = 'formula'
        self._pid = _Pid(
            float(self.get_parameter('kp').value),
            float(self.get_parameter('ki').value),
            float(self.get_parameter('kd').value),
            float(self.get_parameter('i_limit').value),
        )

        self._log_path = _studio_log_path('oms.log')
        raw_mlog = str(self.get_parameter('motion_log').value or '').strip()
        self._motion_log = Path(raw_mlog) if raw_mlog else _studio_log_path('oms_motion.csv')
        self._motion_period = 1.0 / max(2.0, float(self.get_parameter('motion_log_hz').value))
        self._motion_last = 0.0
        self._cmd_text = ''
        self._cmd_target: Optional[float] = None
        self._ticks: Optional[float] = None
        self._home_ticks = 0.0
        self._angle = 0.0
        self._homed = False
        self._home_sw = False
        self._last_duty = 0.0
        self._state = 'boot'
        self._busy = False
        self._cancel = threading.Event()
        self._job_gen = 0
        self._lock = threading.Lock()

        titan_srv = str(self.get_parameter('titan_service').value)
        self._titan = self.create_client(SetData, titan_srv)
        self._motor_cmd = self.create_publisher(
            Float64, str(self.get_parameter('cmd_topic').value), 10)
        self._result_pub = self.create_publisher(String, '/oms/rotation/result', 10)
        self._status_pub = self.create_publisher(String, '/oms/rotation/status', 10)
        self._angle_pub = self.create_publisher(Float64, '/oms/rotation/angle_deg', 10)
        self.create_subscription(
            Float64, str(self.get_parameter('encoder_topic').value),
            self._on_encoder, 10)
        self.create_subscription(
            Bool, str(self.get_parameter('home_limit_topic').value),
            self._on_limit, 10)
        self.create_subscription(String, '/oms/rotation/cmd', self._on_cmd, 10)
        self.create_subscription(
            Float64, '/oms/rotation/set_angle', self._on_set_angle, 10)

        self.create_timer(0.1, self._publish_status)
        self._init_motion_log()
        self.get_logger().info(
            f'OMS rotation  motor=titan1 m_{self._motor}  '
            f'ticks/deg={self._ticks_per_deg:.3f} ({self._tpd_source})  '
            f'duty={self._duty_max:.2f}  '
            f'duty_sign={self._duty_sign:.0f}  home={self.get_parameter("home_limit_topic").value}  '
            f'motion_log={self._motion_log}')
        self._flog(
            'info',
            f'start ticks/deg={self._ticks_per_deg:.3f} duty={self._duty_max:.2f}')
        if bool(self.get_parameter('auto_home').value):
            threading.Thread(target=self._boot_home, daemon=True).start()

    def _flog(self, level: str, msg: str) -> None:
        line = f'{time.strftime("%H:%M:%S")} [{level}] {msg}'
        log = self.get_logger()
        if level == 'error':
            log.error(f'[oms] {msg}')
        elif level == 'warn':
            log.warn(f'[oms] {msg}')
        else:
            log.info(f'[oms] {msg}')
        try:
            self._log_path.parent.mkdir(parents=True, exist_ok=True)
            with open(self._log_path, 'a', encoding='utf-8') as handle:
                handle.write(line + '\n')
        except Exception:
            pass

    def _init_motion_log(self) -> None:
        try:
            self._motion_log.parent.mkdir(parents=True, exist_ok=True)
            with open(self._motion_log, 'w', encoding='utf-8') as handle:
                handle.write(
                    'time,event,cmd,target_deg,ticks,angle_deg,duty,'
                    'home_sw,homed,state\n')
        except Exception as exc:
            self.get_logger().error(f'cannot write {self._motion_log}: {exc}')

    def _mlog(self, event: str, *, force: bool = False) -> None:
        now = time.time()
        if not force and (now - self._motion_last) < self._motion_period:
            return
        self._motion_last = now
        ticks = '' if self._ticks is None else f'{self._ticks:.1f}'
        angle = f'{self._angle_now():.3f}'
        target = '' if self._cmd_target is None else f'{self._cmd_target:.2f}'
        cmd = self._cmd_text.replace(',', ' ')
        line = (
            f'{time.strftime("%H:%M:%S")}.{int((now % 1) * 1000):03d},'
            f'{event},{cmd},{target},{ticks},{angle},'
            f'{self._last_duty:.3f},{int(self._home_sw)},{int(self._homed)},'
            f'{self._state}')
        try:
            with open(self._motion_log, 'a', encoding='utf-8') as handle:
                handle.write(line + '\n')
        except Exception:
            pass
        if force or event not in ('sample',):
            self.get_logger().info(f'[oms_motion] {line}')

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
        if self._ticks is None:
            return float(self._angle)
        if not self._homed:
            return float(self._angle)
        return self._ticks_sign * (
            self._ticks - self._home_ticks) / max(1e-6, self._ticks_per_deg)

    def _publish_status(self) -> None:
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
        self._flog('info' if ok else 'warn', f'result ok={ok} {message}')

    def _wait_titan(self, timeout: float = 30.0) -> bool:
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < timeout:
            if self._titan.wait_for_service(timeout_sec=0.5):
                return True
        return self._titan.service_is_ready()

    def _titan_call(self, params: str, speed: float = 0.0) -> bool:
        if not self._titan.service_is_ready():
            return False
        req = SetData.Request()
        req.params = params
        req.initparams.n_encoder = self._motor
        req.initparams.speed = float(speed)
        fut = self._titan.call_async(req)
        t0 = time.time()
        while rclpy.ok() and not fut.done() and (time.time() - t0) < 1.5:
            time.sleep(0.01)
        return fut.done()

    def _apply_duty(self, duty: float) -> None:
        duty = max(-1.0, min(1.0, float(duty)))
        self._motor_cmd.publish(Float64(data=duty))
        if abs(duty - self._last_duty) < 1e-4:
            return
        self._last_duty = duty
        self._titan_call('set_speed', duty)

    def _stop_motor(self) -> None:
        self._apply_duty(0.0)
        self._last_duty = 0.0
        self._titan_call('set_speed', 0.0)
        self._motor_cmd.publish(Float64(data=0.0))

    def _mark_home(self) -> None:
        self._stop_motor()
        self._titan_call('reset_encoder', 0.0)
        time.sleep(0.05)
        self._home_ticks = float(self._ticks or 0.0)
        self._angle = 0.0
        self._homed = True
        self._state = 'idle'
        self._flog('info', f'homed ticks={self._home_ticks:.1f}')
        self._mlog('homed', force=True)

    def _sleep(self, sec: float) -> bool:
        until = time.time() + max(0.0, sec)
        while rclpy.ok() and not self._cancel.is_set() and time.time() < until:
            time.sleep(self._dt)
        return rclpy.ok() and not self._cancel.is_set()

    def _drive_until(
            self, duty: float, *, until_home: bool, timeout: float,
            gen: int) -> str:
        """Run a constant duty. Returns home | stall | timeout | cancel."""
        self._apply_duty(duty)
        t0 = time.time()
        last_move = time.time()
        last_ticks = self._ticks
        stall_arm = t0 + 1.2
        while rclpy.ok() and not self._cancel.is_set() and gen == self._job_gen:
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
                    self._flog(
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
        t0 = time.time()
        deadline = t0 + self._home_timeout
        sign = self._home_search_sign
        reversals = 0
        self._cmd_text = 'home'
        self._cmd_target = 0.0
        self._flog(
            'info',
            f'home search duty={sign * self._home_duty:+.2f}  '
            f'switch={self._home_sw} ticks={self._ticks}')
        while rclpy.ok() and not self._cancel.is_set() and gen == self._job_gen:
            remain = deadline - time.time()
            if remain <= 0.0:
                self._stop_motor()
                self._state = 'home_failed'
                self._flog('error', 'home timeout — limit switch never pressed')
                return False
            outcome = self._drive_until(
                sign * self._home_duty, until_home=True, timeout=remain, gen=gen)
            if outcome == 'home':
                self._mark_home()
                return True
            if outcome == 'stall':
                reversals += 1
                self._flog('warn', f'home search stalled — reverse ({reversals})')
                self._stop_motor()
                if reversals >= 4:
                    self._state = 'home_failed'
                    self._flog(
                        'error',
                        'encoder not moving on M2 in either direction — '
                        'check Titan1 M2 / raise home_duty / watch '
                        '/titan1/m_2/encoder and /titan1/m_0/limit_rev')
                    return False
                if not self._sleep(0.15):
                    return False
                sign = -sign
                continue
            self._stop_motor()
            if outcome == 'cancel':
                return False
            self._state = 'home_failed'
            return False
        self._stop_motor()
        return False

    def _recover_stall(self, gen: int) -> bool:
        prev = self._last_duty
        self._state = 'stall_recover'
        self._flog('warn', 'stall — reverse to home switch')
        self._stop_motor()
        if not self._sleep(0.12):
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
            self._flog('warn', 'reverse also stalled — flip again')
            self._stop_motor()
            if not self._sleep(0.12):
                return False
            outcome = self._drive_until(
                -duty, until_home=True, timeout=self._home_timeout, gen=gen)
            if outcome == 'home':
                self._mark_home()
                return True
        self._stop_motor()
        self._state = 'home_failed'
        return False

    def _goto(self, target: float, gen: int) -> str:
        target = max(self._min_deg, min(self._max_deg, float(target)))
        if not self._homed:
            if not self._search_home(gen):
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
        self._cmd_text = f'goto {target:+.2f}'
        self._cmd_target = target
        self._flog('info', f'goto {target:+.2f}° from {start_angle:+.2f}°')
        self._mlog('goto_start', force=True)
        while rclpy.ok() and not self._cancel.is_set() and gen == self._job_gen:
            now = time.time()
            if now - t0 >= self._goto_timeout:
                self._stop_motor()
                self._state = 'idle'
                self._mlog('timeout', force=True)
                return 'timeout'
            angle = self._angle_now()
            err = target - angle
            dt = max(self._dt, now - last)
            last = now
            if abs(err) <= self._tol:
                self._apply_duty(0.0)
                if in_tol_t is None:
                    in_tol_t = now
                elif now - in_tol_t >= self._settle:
                    self._stop_motor()
                    self._state = 'idle'
                    self._mlog('arrived', force=True)
                    return 'ok'
            else:
                in_tol_t = None
                raw = self._duty_sign * self._pid.update(err, dt)
                duty = _shape_duty(raw, self._duty_min, self._duty_max)
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
                        self._flog(
                            'warn',
                            f'stall at software {angle:+.1f}° ticks={ticks:.0f} '
                            f'(started {start_angle:+.1f}° ticks={start_ticks}) '
                            f'target {target:+.1f}° traveled={traveled:.1f}°')
                        if traveled >= self._stall_travel:
                            self._stop_motor()
                            self._state = 'idle'
                            self._flog(
                                'warn',
                                'mechanical end stop — not rehoming. '
                                'If this pose is physically 90°, measure ticks '
                                'and set ticks_per_deg = abs(ticks)/90 in oms.yaml')
                            self._mlog('at_stop', force=True)
                            return 'at_stop'
                        if self._recover_stall(gen):
                            self._mlog('rehomed', force=True)
                            return 'rehomed'
                        return 'stall recover failed'
            time.sleep(self._dt)
        self._stop_motor()
        return 'cancel'

    def _boot_home(self) -> None:
        self._state = 'wait_titan'
        if not self._wait_titan():
            self._state = 'home_failed'
            self._flog('error', 'titan1 service not ready — cannot home')
            return
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < 10.0:
            if self._ticks is not None:
                break
            time.sleep(0.05)
        if self._ticks is None:
            self._flog('warn', 'no /titan1/m_2/encoder yet — homing anyway')
        time.sleep(0.4)
        with self._lock:
            self._job_gen += 1
            gen = self._job_gen
            self._busy = True
            self._cancel.clear()
        ok = self._search_home(gen)
        self._busy = False
        self._pub_result('', ok, 'homed' if ok else 'boot home failed')
        self._mlog('boot_home', force=True)

    def _on_set_angle(self, msg: Float64) -> None:
        cmd = {
            'cmd_id': f'set_{int(time.time() * 1000)}',
            'mode': 'goto',
            'angle_deg': float(msg.data),
        }
        self._on_cmd(String(data=json.dumps(cmd)))

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
        cmd_id = str(cmd.get('cmd_id') or '')
        self._cancel.set()
        with self._lock:
            self._job_gen += 1
            gen = self._job_gen
        self._cmd_text = f'{mode} {cmd.get("angle_deg", "")}'.strip()
        if mode == 'goto':
            try:
                self._cmd_target = float(cmd.get('angle_deg'))
            except (TypeError, ValueError):
                self._cmd_target = None
        self._mlog('cmd', force=True)
        threading.Thread(
            target=self._run_cmd, args=(cmd_id, mode, cmd, gen), daemon=True
        ).start()

    def _run_cmd(self, cmd_id: str, mode: str, cmd: dict, gen: int) -> None:
        time.sleep(0.02)
        self._cancel.clear()
        if gen != self._job_gen:
            return
        self._busy = True
        try:
            if mode in ('stop', 'cancel'):
                self._stop_motor()
                self._state = 'idle'
                self._pub_result(cmd_id, True, 'stopped')
                return
            if mode == 'home':
                ok = self._search_home(gen)
                self._pub_result(cmd_id, ok, 'homed' if ok else 'home failed')
                return
            if mode != 'goto':
                self._pub_result(cmd_id, False, f'unknown mode {mode}')
                return
            try:
                target = float(cmd.get('angle_deg'))
            except (TypeError, ValueError):
                self._pub_result(cmd_id, False, 'angle_deg required')
                return
            outcome = self._goto(target, gen)
            ok = outcome in ('ok', 'rehomed', 'at_stop')
            self._pub_result(cmd_id, ok, outcome)
        finally:
            if gen == self._job_gen:
                self._busy = False
                if self._state not in ('home_failed',):
                    self._state = 'idle' if self._homed else self._state
                self._stop_motor()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OmsRotationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_motor()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
