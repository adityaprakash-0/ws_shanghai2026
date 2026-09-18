#!/usr/bin/env python3
"""Per-waypoint OMS sequence runner for Navigation / Localisation missions.

Dashboard stores the steps on each waypoint action, e.g.::

    {
      "type": "oms",                 # or "align_move_oms"
      "sequence": ["home", "lift_goto", "gripper_open", "wait"],
      "lift_mm": 40.0,
      "linear_mm": 80.0,
      "rotation_deg": 0.0,
      "wait_sec": 1.0,
      "timeout_sec": 120.0
    }

Supported step names (comma-separated string also OK)::

    home
    rotation_goto | rotation_home
    lift_goto | lift_home
    linear_goto | linear_home
    vertical_up | vertical_down | vertical_home
    rotation_servo_out | rotation_servo_in | rotation_servo_home
    gripper_open | gripper_close | gripper_home
    wait
"""

from __future__ import annotations

import json
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

LogFn = Callable[[str, str], None]


ALLOWED_STEPS = frozenset({
    'home',
    'rotation_goto', 'rotation_home',
    'lift_goto', 'lift_home',
    'linear_goto', 'linear_home',
    'vertical_up', 'vertical_down', 'vertical_home',
    'rotation_servo_out', 'rotation_servo_in', 'rotation_servo_home',
    'gripper_open', 'gripper_close', 'gripper_home',
    'wait',
})


def normalize_sequence(raw) -> List[str]:
    if raw is None:
        return []
    if isinstance(raw, str):
        raw = raw.replace('→', ',').replace(';', ',').split(',')
    out: List[str] = []
    for step in list(raw):
        name = str(step).strip().lower()
        if name:
            out.append(name)
    return out


def axis_cmd(
        pub, result_store: Dict[str, Any], axis: str, mode: str,
        timeout: float, extra: Optional[dict] = None,
        log: Optional[LogFn] = None,
) -> Tuple[bool, str]:
    from std_msgs.msg import String
    _log = log or (lambda _m, _l='info': None)
    cmd_id = f'wp_oms_{axis}_{int(time.time() * 1000)}'
    result_store.pop(axis, None)
    payload: Dict[str, Any] = {'cmd_id': cmd_id, 'mode': mode}
    if extra:
        payload.update(extra)
    pub.publish(String(data=json.dumps(payload)))
    _log(f'{axis} mode={mode} id={cmd_id}', 'info')
    deadline = time.time() + max(2.0, float(timeout))
    while time.time() < deadline:
        res = result_store.get(axis)
        if isinstance(res, dict) and str(res.get('cmd_id') or '') == cmd_id:
            ok = bool(res.get('success'))
            msg = str(res.get('message') or '')
            _log(f'{axis} ok={ok} {msg}', 'info' if ok else 'warn')
            return ok, msg
        time.sleep(0.05)
    return False, f'{axis} timeout'


def run_wp_oms_sequence(
        *,
        pubs: Dict[str, Any],
        result_store: Dict[str, Any],
        sequence,
        lift_mm: float = 0.0,
        linear_mm: float = 0.0,
        rotation_deg: float = 0.0,
        wait_sec: float = 1.0,
        axis_timeout_sec: float = 60.0,
        home_timeout_sec: float = 180.0,
        log: Optional[LogFn] = None,
        cancel_check: Optional[Callable[[], bool]] = None,
) -> Tuple[bool, str]:
    """Execute a dashboard-defined OMS step list. Blocking."""
    _log = log or (lambda _m, _l='info': None)
    steps = normalize_sequence(sequence)
    if not steps:
        return False, 'empty OMS sequence'
    unknown = [s for s in steps if s not in ALLOWED_STEPS]
    if unknown:
        return False, (
            f'unknown OMS steps {unknown}; '
            f'allowed={sorted(ALLOWED_STEPS)}')

    lift_mm = max(0.0, min(300.0, float(lift_mm)))
    linear_mm = max(0.0, min(300.0, float(linear_mm)))
    rotation_deg = max(-120.0, min(120.0, float(rotation_deg)))
    wait_sec = max(0.0, min(60.0, float(wait_sec)))
    axis_t = max(5.0, float(axis_timeout_sec))
    home_t = max(10.0, float(home_timeout_sec))

    def _cancelled() -> bool:
        return bool(cancel_check and cancel_check())

    for index, step in enumerate(steps, 1):
        if _cancelled():
            return False, 'OMS sequence cancelled'
        _log(f'OMS {index}/{len(steps)} — {step}', 'info')
        ok = True
        msg = ''
        if step == 'home':
            ok, msg = axis_cmd(
                pubs['home'], result_store, 'home', 'home', home_t, log=_log)
        elif step == 'rotation_goto':
            ok, msg = axis_cmd(
                pubs['rotation'], result_store, 'rotation', 'goto', axis_t,
                {'angle_deg': rotation_deg}, log=_log)
        elif step == 'rotation_home':
            ok, msg = axis_cmd(
                pubs['rotation'], result_store, 'rotation', 'home', axis_t,
                log=_log)
        elif step == 'lift_goto':
            ok, msg = axis_cmd(
                pubs['lift'], result_store, 'lift', 'goto', axis_t,
                {'pos_mm': lift_mm}, log=_log)
        elif step == 'lift_home':
            ok, msg = axis_cmd(
                pubs['lift'], result_store, 'lift', 'home', axis_t, log=_log)
        elif step == 'linear_goto':
            ok, msg = axis_cmd(
                pubs['linear'], result_store, 'linear', 'goto', axis_t,
                {'pos_mm': linear_mm}, log=_log)
        elif step == 'linear_home':
            ok, msg = axis_cmd(
                pubs['linear'], result_store, 'linear', 'home', axis_t,
                log=_log)
        elif step in ('vertical_up', 'vertical_down', 'vertical_home'):
            mode = {
                'vertical_up': 'up',
                'vertical_down': 'down',
                'vertical_home': 'home',
            }[step]
            ok, msg = axis_cmd(
                pubs['vertical'], result_store, 'vertical', mode, axis_t,
                log=_log)
        elif step in (
                'rotation_servo_out', 'rotation_servo_in',
                'rotation_servo_home'):
            mode = {
                'rotation_servo_out': 'out',
                'rotation_servo_in': 'in',
                'rotation_servo_home': 'home',
            }[step]
            ok, msg = axis_cmd(
                pubs['rotation_servo'], result_store, 'rotation_servo', mode,
                axis_t, log=_log)
        elif step in ('gripper_open', 'gripper_close', 'gripper_home'):
            mode = step.removeprefix('gripper_')
            ok, msg = axis_cmd(
                pubs['gripper'], result_store, 'gripper', mode, axis_t,
                log=_log)
        elif step == 'wait':
            deadline = time.time() + wait_sec
            while time.time() < deadline:
                if _cancelled():
                    return False, 'OMS sequence cancelled'
                time.sleep(0.05)
            ok, msg = True, f'waited {wait_sec:.1f}s'
        else:
            return False, f'unhandled step {step}'
        if not ok:
            return False, f'step {step} failed: {msg}'
        time.sleep(0.08)
    return True, f'OMS sequence complete ({len(steps)} steps)'
