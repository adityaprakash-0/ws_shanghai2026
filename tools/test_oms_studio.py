#!/usr/bin/env python3
"""Offline checks for OMS Software pose math and yaml patching."""

from __future__ import annotations

import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'oms'))

from oms.kinematics import gripper_xyz_m, oms_analysis  # noqa: E402
from oms.config_io import public_config, save_oms_yaml  # noqa: E402


def almost(a, b, eps=1e-6):
    return abs(a - b) <= eps


def test_home_pose():
    # Home: linear rear limit at 220 mm, lift at upper (455 mm Z).
    x, y, z = gripper_xyz_m(0, 0, 0)
    assert almost(x, 0.220) and almost(y, 0.0) and almost(z, 0.455), (x, y, z)


def test_lift_down():
    # Floor: World Z = 155 mm at full lift travel (300 mm down).
    x, y, z = gripper_xyz_m(0, 300, 0)
    assert almost(z, 0.155) and almost(x, 0.220), (x, y, z)


def test_linear_forward():
    # Stroke 100 from home → World X = 320 mm.
    x, y, z = gripper_xyz_m(0, 0, 100)
    assert almost(x, 0.320) and almost(y, 0.0), (x, y, z)


def test_linear_clamp_to_max_reach():
    x, y, z = gripper_xyz_m(0, 0, 500)
    assert almost(x, 0.330), (x, y, z)


def test_rotate_90_acw():
    x, y, z = gripper_xyz_m(90, 0, 0)
    assert almost(x, 0.0, 1e-9) and almost(y, 0.220), (x, y, z)


def test_rotate_90_cw():
    x, y, z = gripper_xyz_m(-90, 0, 0)
    assert almost(x, 0.0, 1e-9) and almost(y, -0.220), (x, y, z)


def test_analysis_keys():
    info = oms_analysis(30, 50, 20)
    assert info['front'] == '+X'
    assert 'x_m' in info and info['yaw_deg'] == 30.0
    assert info['z_top_mm'] == 455.0


def test_yaml_roundtrip():
    src = ROOT / 'src' / 'oms' / 'config' / 'oms.yaml'
    text = src.read_text(encoding='utf-8')
    tmp = Path(tempfile.mkdtemp()) / 'oms.yaml'
    tmp.write_text(text, encoding='utf-8')
    save_oms_yaml({'oms_rotation': {'duty': 0.11}}, paths=[tmp])
    from oms.config_io import load_oms_yaml
    loaded = load_oms_yaml(tmp)
    duty = loaded['oms_rotation']['ros__parameters']['duty']
    assert abs(float(duty) - 0.11) < 1e-9, duty
    assert 'OMS — all mechanisms' in tmp.read_text(encoding='utf-8')
    cfg = public_config(loaded)
    assert 'oms_viz' in cfg
    assert cfg['oms_viz']['lift_travel_mm'] == 300.0
    assert cfg['oms_viz']['linear_home_x_mm'] == 220.0
    save_oms_yaml({
        'oms_home': {
            'sequence': ['lift', 'linear', 'gripper'],
        },
    }, paths=[tmp])
    loaded2 = load_oms_yaml(tmp)
    seq = loaded2['oms_home']['ros__parameters']['sequence']
    assert list(seq) == ['lift', 'linear', 'gripper'], seq
    cfg2 = public_config(loaded2)
    assert cfg2['oms_home']['sequence'] == ['lift', 'linear', 'gripper']


def _js_balanced(path: Path) -> None:
    text = path.read_text(encoding='utf-8')
    stack = []
    pairs = {')': '(', ']': '[', '}': '{'}
    in_str = None
    esc = False
    for ch in text:
        if in_str:
            if esc:
                esc = False
            elif ch == '\\':
                esc = True
            elif ch == in_str:
                in_str = None
            continue
        if ch in ('"', "'"):
            in_str = ch
            continue
        if ch in '([{':
            stack.append(ch)
        elif ch in ')]}':
            if not stack or stack[-1] != pairs[ch]:
                raise AssertionError(f'{path.name} unbalanced near {ch}')
            stack.pop()
    if stack:
        raise AssertionError(f'{path.name} unclosed {stack[-1]}')


def test_js_balanced():
    root = ROOT / 'laptop' / 'skill23_studio' / 'js'
    for name in ('app.js', 'api.js', 'oms.js', 'oms3d.js'):
        _js_balanced(root / name)


def test_commission_launch_parses():
    path = ROOT / 'src' / 'skill23_studio' / 'launch' / 'studio_commission.launch.py'
    compile(path.read_text(encoding='utf-8'), str(path), 'exec')


def test_js_pose_matches_python():
    js = (ROOT / 'laptop' / 'skill23_studio' / 'js' / 'oms3d.js').read_text(
        encoding='utf-8')
    assert 'Math.cos(yaw)' in js
    assert 'Math.sin(yaw)' in js
    html = (ROOT / 'laptop' / 'skill23_studio' / 'oms.html').read_text(
        encoding='utf-8')
    assert 'id="omsCanvas"' in html
    assert 'id="homeSeq"' in html
    assert 'Home sequence' in html
    assert 'Run OMS' in html
    oms_js = (ROOT / 'laptop' / 'skill23_studio' / 'js' / 'oms.js').read_text(
        encoding='utf-8')
    assert 'HOME_STEPS' in oms_js
    assert "axis: 'home'" in oms_js
    assert 'oms_home' in oms_js
    app = (ROOT / 'laptop' / 'skill23_studio' / 'js' / 'app.js').read_text(
        encoding='utf-8')
    assert 'function setConnected(ok)' in app
    assert 'function openOmsSoftware()' in app
    assert 'Connect to the Pi first' in app
    assert 'alertIfBusy' in app


def main():
    tests = [
        test_home_pose, test_lift_down, test_linear_forward,
        test_linear_clamp_to_max_reach,
        test_rotate_90_acw, test_rotate_90_cw, test_analysis_keys,
        test_yaml_roundtrip, test_js_balanced, test_commission_launch_parses,
        test_js_pose_matches_python,
    ]
    failed = 0
    for fn in tests:
        try:
            fn()
            print(f'OK  {fn.__name__}')
        except Exception as exc:
            failed += 1
            print(f'FAIL {fn.__name__}: {exc}')
    if failed:
        sys.exit(1)
    print('all oms studio kinematics/config tests passed')


if __name__ == '__main__':
    main()
