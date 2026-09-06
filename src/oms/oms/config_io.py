"""Read / patch oms.yaml so Studio can save teleop speeds and viz dimensions."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Dict, Iterable, Optional

import yaml

_WS_YAML = Path(os.path.expanduser('~/skill23_ws/src/oms/config/oms.yaml'))
_PKG_YAML = Path(__file__).resolve().parent.parent / 'config' / 'oms.yaml'

EDITABLE = {
    'oms_home': (
        'auto_home', 'startup_delay_sec', 'step_timeout_sec',
        'inter_step_sec', 'sequence',
    ),
    'oms_rotation': (
        'duty', 'duty_min', 'home_duty', 'min_deg', 'max_deg',
        'angle_tol_deg', 'goto_timeout_sec', 'kp',
    ),
    'oms_lift': (
        'duty', 'duty_down', 'home_duty', 'min_mm', 'max_mm',
        'pos_tol_mm', 'goto_timeout_sec',
    ),
    'oms_linear': (
        'speed', 'speed_min', 'home_speed', 'min_mm', 'max_mm',
        'pos_tol_mm', 'mm_per_rev', 'ticks_per_mm',
    ),
    'oms_vertical': ('up_deg', 'down_deg', 'settle_sec'),
    'oms_rotation_servo': ('home_deg', 'alt_deg', 'settle_sec'),
    'oms_gripper': ('home_deg', 'alt_deg', 'settle_sec'),
    'oms_viz': (
        'lift_travel_mm', 'z_top_mm', 'z_floor_mm',
        'linear_travel_mm', 'linear_home_x_mm', 'linear_max_reach_mm',
        'rail_height_mm', 'turntable_diameter_mm', 'camera_tilt_deg',
        'gripper_servo_mm', 'gripper_length_mm',
    ),
}


def yaml_paths() -> list[Path]:
    paths: list[Path] = []
    for path in (_WS_YAML, _PKG_YAML):
        if path.is_file() and path not in paths:
            paths.append(path)
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('oms')) / 'config' / 'oms.yaml'
        if share.is_file() and share not in paths:
            paths.append(share)
    except Exception:
        pass
    return paths


def load_oms_yaml(path: Optional[Path] = None) -> Dict[str, Any]:
    target = path or (yaml_paths()[0] if yaml_paths() else _WS_YAML)
    with open(target, 'r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        return {}
    return data


def public_config(data: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    raw = data if data is not None else load_oms_yaml()
    out: Dict[str, Any] = {}
    for node, keys in EDITABLE.items():
        params = ((raw.get(node) or {}).get('ros__parameters') or {})
        out[node] = {k: params.get(k) for k in keys if k in params}
    return out


def _replace_key_in_section(text: str, section: str, key: str, value: Any) -> str:
    lines = text.splitlines(keepends=True)
    in_section = False
    in_params = False
    depth_ok = False
    rendered = _render_yaml_value(value)
    for i, line in enumerate(lines):
        stripped = line.lstrip()
        indent = len(line) - len(stripped)
        if indent == 0 and stripped.startswith(f'{section}:'):
            in_section = True
            in_params = False
            continue
        if in_section and indent == 0 and stripped and not stripped.startswith('#'):
            if not stripped.startswith(f'{section}:'):
                in_section = False
                in_params = False
        if in_section and stripped.startswith('ros__parameters:'):
            in_params = True
            continue
        if in_section and in_params and indent <= 2 and stripped and not stripped.startswith('#'):
            if not stripped.startswith('ros__parameters:') and indent == 0:
                in_params = False
        if in_section and in_params and stripped.startswith(f'{key}:'):
            prefix = line[:line.find(key)]
            nl = '\n' if line.endswith('\n') else ''
            lines[i] = f'{prefix}{key}: {rendered}{nl}'
            depth_ok = True
            break
    if not depth_ok:
        raise KeyError(f'{section}.{key} not found in oms.yaml')
    return ''.join(lines)


def _render_yaml_value(value: Any) -> str:
    if isinstance(value, bool):
        return 'true' if value else 'false'
    # Always emit floats with a decimal so ROS double params stay doubles.
    if isinstance(value, float) or (isinstance(value, int) and not isinstance(value, bool)):
        num = float(value)
        if num == int(num) and abs(num) < 1e12:
            return f'{int(num)}.0' if num == int(num) else str(num)
        text = f'{num:.6g}'
        if '.' not in text and 'e' not in text.lower():
            text += '.0'
        return text
    if isinstance(value, (list, tuple)):
        return '[' + ', '.join(str(item) for item in value) + ']'
    return str(value)


def save_oms_yaml(
        updates: Dict[str, Dict[str, Any]],
        paths: Optional[Iterable[Path]] = None) -> Dict[str, Any]:
    """Patch editable keys in-place so comments stay. Writes every existing copy."""
    targets = list(paths) if paths is not None else yaml_paths()
    if not targets:
        raise FileNotFoundError('oms.yaml not found')
    written: list[str] = []
    merged: Dict[str, Any] = {}
    for path in targets:
        text = path.read_text(encoding='utf-8')
        for section, keys in updates.items():
            if section not in EDITABLE:
                raise ValueError(f'unknown section {section}')
            allowed = set(EDITABLE[section])
            for key, value in (keys or {}).items():
                if key not in allowed:
                    raise ValueError(f'cannot edit {section}.{key}')
                text = _replace_key_in_section(text, section, key, value)
        path.write_text(text, encoding='utf-8')
        written.append(str(path))
    merged = public_config(load_oms_yaml(targets[0]))
    return {'ok': True, 'written': written, 'config': merged}
