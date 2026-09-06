#!/usr/bin/env python3
"""Shared OMS pick sequence used by detect_align (mode=oms_pick) and tests.

Sequence (matches detect_align_oms_test bench):
  1. Color detect (N frames) → best conf
  2. OMS rotate to bearing target (retry until tolerance)
  3. Post-rotate: detect + depth → linear_mm
  4. Refine OMS if |bearing| > yaw_tol
  5. rotation_servo in → vertical up → gripper open
  6. Lift goto lift_mm → linear goto → gripper close → linear home
"""

from __future__ import annotations

import json
import time
from typing import Any, Callable, Dict, Optional, Tuple

LogFn = Callable[[str, str], None]


def depth_to_linear_mm(
        depth_z_m: float, cam_grip_mm: float, fwd_offset_mm: float,
        linear_max_mm: float) -> float:
    raw = float(depth_z_m) * 1000.0 - float(cam_grip_mm) + float(fwd_offset_mm)
    return max(0.0, min(float(linear_max_mm), raw))


def axis_cmd(
        pub, result_store: Dict[str, Any], axis: str, mode: str,
        timeout: float, extra: Optional[dict] = None,
        log: Optional[LogFn] = None,
) -> Tuple[bool, str]:
    from std_msgs.msg import String
    _log = log or (lambda _m, _l='info': None)
    cmd_id = f'da_{axis}_{int(time.time() * 1000)}'
    result_store.pop(axis, None)
    payload: Dict[str, Any] = {'cmd_id': cmd_id, 'mode': mode}
    if extra:
        payload.update(extra)
    pub.publish(String(data=json.dumps(payload)))
    _log(f'{axis.upper()} mode={mode} id={cmd_id}', 'info')
    deadline = time.time() + timeout
    while time.time() < deadline:
        res = result_store.get(axis)
        if isinstance(res, dict) and str(res.get('cmd_id') or '') == cmd_id:
            ok = bool(res.get('success'))
            msg = str(res.get('message') or '')
            _log(f'{axis.upper()} ok={ok} {msg}', 'info' if ok else 'warn')
            return ok, msg
        time.sleep(0.05)
    return False, f'{axis} timeout'


def run_pick_arm_and_grip(
        *,
        lift_pub, linear_pub, rot_servo_pub, vert_pub, grip_pub,
        result_store: Dict[str, Any],
        lift_mm: float,
        linear_mm: float,
        linear_max_mm: float,
        lift_timeout: float,
        linear_timeout: float,
        servo_timeout: float,
        linear_enabled: bool = True,
        log: Optional[LogFn] = None,
) -> Tuple[bool, str, Dict[str, Any]]:
    """After OMS yaw+depth: arm prep, lift, linear, grip, linear home."""
    _log = log or (lambda _m, _l='info': None)
    extra: Dict[str, Any] = {
        'pick_lift_mm': round(float(lift_mm), 2),
        'linear_pos_mm': round(float(linear_mm), 2),
    }
    _log('ARM PREP — rotation_servo in, vertical up, gripper open', 'info')
    for pub, axis, mode in (
            (rot_servo_pub, 'rotation_servo', 'in'),
            (vert_pub, 'vertical', 'up'),
            (grip_pub, 'gripper', 'open'),
    ):
        ok, msg = axis_cmd(
            pub, result_store, axis, mode, servo_timeout, log=_log)
        if not ok:
            return False, f'{axis} {mode} failed: {msg}', extra
        time.sleep(0.12)

    lift_mm = max(0.0, min(300.0, float(lift_mm)))
    _log(f'LIFT goto {lift_mm:.0f} mm down from home', 'info')
    ok, msg = axis_cmd(
        lift_pub, result_store, 'lift', 'goto', lift_timeout,
        {'pos_mm': lift_mm}, log=_log)
    if not ok:
        return False, f'lift failed: {msg}', extra

    if linear_enabled:
        pos = max(0.0, min(float(linear_max_mm), float(linear_mm)))
        ok, msg = axis_cmd(
            linear_pub, result_store, 'linear', 'goto', linear_timeout,
            {'pos_mm': pos}, log=_log)
        if not ok:
            return False, f'linear failed: {msg}', extra
    else:
        _log(f'linear disabled — would goto {float(linear_mm):.1f} mm', 'info')

    _log('GRIPPER close', 'info')
    ok, msg = axis_cmd(
        grip_pub, result_store, 'gripper', 'close', servo_timeout, log=_log)
    if not ok:
        ok2, msg2 = axis_cmd(
            grip_pub, result_store, 'gripper', 'home', servo_timeout, log=_log)
        if not ok2:
            return False, f'gripper close failed: {msg} / {msg2}', extra

    if linear_enabled:
        _log('LINEAR home — retract after grip', 'info')
        ok, msg = axis_cmd(
            linear_pub, result_store, 'linear', 'home', linear_timeout, log=_log)
        if not ok:
            return False, f'linear home failed: {msg}', extra

    extra['pick_ok'] = True
    return True, 'pick complete', extra
