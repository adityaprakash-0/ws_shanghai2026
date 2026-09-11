#!/usr/bin/env python3
"""Shared OMS pick sequence used by detect_align (mode=oms_pick) and tests.

Order:
  1. OMS home
  2. rotation_servo out → linear pre-extend (10 cm) → lift to pick_lift_mm
  3. Color detect → OMS yaw → depth → absolute linear_mm
       remain = depth_mm − (camera_to_gripper_mm + pre_extend_mm) + offset
       absolute = pre_extend_mm + remain
                = depth_mm − camera_to_gripper_mm + offset
  4. gripper open → linear goto target → gripper close → linear home
"""

from __future__ import annotations

import json
import time
from typing import Any, Callable, Dict, Optional, Tuple

LogFn = Callable[[str, str], None]


def depth_to_linear_mm(
        depth_z_m: float, cam_grip_mm: float, fwd_offset_mm: float,
        linear_max_mm: float, pre_extend_mm: float = 0.0,
) -> float:
    """Absolute linear target from depth (mm from rear home).

    From the pre-extend standoff:
      remain = depth_mm − (cam_grip + pre_extend) + offset
      absolute = pre_extend + remain
    """
    depth_mm = float(depth_z_m) * 1000.0
    pre = max(0.0, float(pre_extend_mm))
    remain = (
        depth_mm
        - (float(cam_grip_mm) + pre)
        + float(fwd_offset_mm))
    absolute = pre + remain
    return max(0.0, min(float(linear_max_mm), absolute))


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


def _step_names(raw, default) -> list[str]:
    """Normalize a YAML string-array (also accept a comma-separated string)."""
    if raw is None:
        raw = default
    if isinstance(raw, str):
        raw = raw.replace('→', ',').split(',')
    return [str(step).strip().lower() for step in list(raw) if str(step).strip()]


def run_pick_pre_detect(
        *,
        lift_pub, linear_pub, rot_servo_pub,
        result_store: Dict[str, Any],
        lift_mm: float,
        pre_extend_mm: float,
        linear_max_mm: float,
        lift_timeout: float,
        linear_timeout: float,
        servo_timeout: float,
        linear_enabled: bool = True,
        sequence=None,
        log: Optional[LogFn] = None,
) -> Tuple[bool, str, Dict[str, Any]]:
    """Run the YAML-selected steps after full home and before camera look."""
    _log = log or (lambda _m, _l='info': None)
    lift_mm = max(0.0, min(300.0, float(lift_mm)))
    pre = max(0.0, min(float(linear_max_mm), float(pre_extend_mm)))
    steps = _step_names(
        sequence, ('rotation_servo_out', 'linear_pre_extend', 'lift_goto'))
    extra: Dict[str, Any] = {
        'pick_lift_mm': round(lift_mm, 2),
        'linear_pre_extend_mm': round(pre, 2),
        'linear_start_mm': 0.0,
        'pre_look_sequence': list(steps),
    }
    allowed = {
        'rotation_servo_out', 'rotation_servo_in',
        'lift_goto', 'lift_home',
        'linear_pre_extend', 'linear_home',
    }
    for index, step in enumerate(steps, 1):
        _log(f'PRE {index}/{len(steps)} — {step}', 'info')
        if step not in allowed:
            return False, (
                f'unknown pre_look_sequence step {step!r}; '
                f'use {", ".join(sorted(allowed))}'), extra
        if step in ('rotation_servo_out', 'rotation_servo_in'):
            mode = 'out' if step.endswith('_out') else 'home'
            ok, msg = axis_cmd(
                rot_servo_pub, result_store, 'rotation_servo', mode,
                servo_timeout, log=_log)
        elif step in ('lift_goto', 'lift_home'):
            mode = 'goto' if step.endswith('_goto') else 'home'
            fields = {'pos_mm': lift_mm} if mode == 'goto' else None
            ok, msg = axis_cmd(
                lift_pub, result_store, 'lift', mode, lift_timeout,
                fields, log=_log)
        elif step == 'linear_pre_extend':
            if not linear_enabled:
                _log(f'linear disabled — would pre-extend {pre:.0f} mm', 'info')
                continue
            ok, msg = axis_cmd(
                linear_pub, result_store, 'linear', 'goto', linear_timeout,
                {'pos_mm': pre}, log=_log)
            if ok:
                extra['linear_start_mm'] = round(pre, 2)
        else:  # linear_home
            if not linear_enabled:
                _log('linear disabled — would home', 'info')
                continue
            ok, msg = axis_cmd(
                linear_pub, result_store, 'linear', 'home', linear_timeout,
                log=_log)
            if ok:
                extra['linear_start_mm'] = 0.0
        if not ok:
            return False, f'pre-look step {step} failed: {msg}', extra
        time.sleep(0.12)

    return True, 'pre-look sequence complete', extra


def run_pick_post_depth(
        *,
        linear_pub, grip_pub,
        result_store: Dict[str, Any],
        linear_mm: float,
        pre_extend_mm: float,
        linear_max_mm: float,
        linear_timeout: float,
        servo_timeout: float,
        linear_enabled: bool = True,
        sequence=None,
        log: Optional[LogFn] = None,
) -> Tuple[bool, str, Dict[str, Any]]:
    """Run the YAML-selected steps after alignment and depth measurement."""
    _log = log or (lambda _m, _l='info': None)
    pre = max(0.0, min(float(linear_max_mm), float(pre_extend_mm)))
    target = max(0.0, min(float(linear_max_mm), float(linear_mm)))
    steps = _step_names(
        sequence, ('gripper_open', 'linear_depth', 'gripper_close', 'linear_home'))
    # Signed move from the position established by pre_look_sequence.
    remain = target - pre
    extra: Dict[str, Any] = {
        'linear_pos_mm': round(target, 2),
        'linear_pre_extend_mm': round(pre, 2),
        'linear_remain_mm': round(remain, 2),
        'post_look_sequence': list(steps),
    }
    allowed = {
        'gripper_open', 'gripper_close', 'gripper_home',
        'linear_depth', 'linear_home',
    }
    for index, step in enumerate(steps, 1):
        _log(f'POST {index}/{len(steps)} — {step}', 'info')
        if step not in allowed:
            return False, (
                f'unknown post_look_sequence step {step!r}; '
                f'use {", ".join(sorted(allowed))}'), extra
        if step.startswith('gripper_'):
            mode = step.removeprefix('gripper_')
            ok, msg = axis_cmd(
                grip_pub, result_store, 'gripper', mode, servo_timeout, log=_log)
            if not ok and step == 'gripper_close':
                ok2, msg2 = axis_cmd(
                    grip_pub, result_store, 'gripper', 'home',
                    servo_timeout, log=_log)
                if not ok2:
                    return False, (
                        f'post-look step {step} failed: {msg} / {msg2}'), extra
                ok, msg = True, msg2
        elif step == 'linear_depth':
            if not linear_enabled:
                _log(f'linear disabled — would goto {target:.1f} mm', 'info')
                continue
            ok, msg = axis_cmd(
                linear_pub, result_store, 'linear', 'goto', linear_timeout,
                {'pos_mm': target}, log=_log)
        else:  # linear_home
            if not linear_enabled:
                _log('linear disabled — would home', 'info')
                continue
            ok, msg = axis_cmd(
                linear_pub, result_store, 'linear', 'home', linear_timeout,
                log=_log)
        if not ok:
            return False, f'post-look step {step} failed: {msg}', extra
        time.sleep(0.12)

    extra['pick_ok'] = True
    return True, 'post-look sequence complete', extra


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
        do_lift: bool = False,
        pre_extend_mm: float = 100.0,
        log: Optional[LogFn] = None,
) -> Tuple[bool, str, Dict[str, Any]]:
    """Backward-compatible wrapper: post-depth grip + linear only.

    Pre-detect (out / 10 cm / lift) must already have run. ``do_lift`` ignored.
    """
    _ = (lift_pub, rot_servo_pub, vert_pub, lift_mm, lift_timeout, do_lift)
    return run_pick_post_depth(
        linear_pub=linear_pub,
        grip_pub=grip_pub,
        result_store=result_store,
        linear_mm=linear_mm,
        pre_extend_mm=pre_extend_mm,
        linear_max_mm=linear_max_mm,
        linear_timeout=linear_timeout,
        servo_timeout=servo_timeout,
        linear_enabled=linear_enabled,
        log=log,
    )
