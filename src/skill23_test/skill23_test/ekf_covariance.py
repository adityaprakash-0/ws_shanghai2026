#!/usr/bin/env python3
"""Build robot_localization P / Q diagonals from skill23_test calibration JSONs.

Edit the two paths below, then:

  ros2 run skill23_test ekf_covariance
  # or: python3 -m skill23_test.ekf_covariance

Writes reports/ekf_<robot>/ekf_covariance.md (copy-paste YAML) plus .json.

Holonomic (mecanum, xdrive): two blocks — full + _diff (vy unused).
Skid-steer (grip, allterrain): one block (no vy).
"""

from __future__ import annotations

import json
import math
import statistics
import sys
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from skill23_test.report_paths import get_reports_dir

# ---------------------------------------------------------------------------
# Paths — edit these to the reports you want to convert.
# IMU can be the xdrive file when the same IMU is used on another chassis.
# ---------------------------------------------------------------------------
REPORTS = get_reports_dir()

ODOM_REPORT = REPORTS / 'allterrain_odom_cal_20260825_140730.json'
IMU_REPORT = REPORTS / 'xdrive_imu_cal_20260823_185134.json'

# ODOM_REPORT = REPORTS / 'grip_odom_cal_20260824_210355_corrected.json'
# ODOM_REPORT = REPORTS / 'mecanum_odom_cal_20260824_194611.json'
# ODOM_REPORT = REPORTS / 'xdrive_odom_cal_20260823_183744.json'
# IMU_REPORT = REPORTS / 'xdrive_imu_cal_20260823_185134.json'

HOLONOMIC = {'mecanum', 'xdrive'}
DIFF_ONLY = {'grip', 'allterrain'}
UNUSED = 1.0e6
# Floor so Q is not tighter than typical wheel pose R (4e-4).
POSE_R_FLOOR = 4.0e-4
VX_Q_FLOOR = 8.0e-3
YAW_P = 8.0e-3
YAW_Q = 3.0e-3
VYW_P = 1.0e-3
VYW_Q = 1.0e-3
VEL_P = 1.0e-3

# robot_localization 15-state order
STATE = (
    'x', 'y', 'z', 'roll', 'pitch', 'yaw',
    'vx', 'vy', 'vz', 'vroll', 'vpitch', 'vyaw',
    'ax', 'ay', 'az',
)


def _load(path: Path) -> Dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f'Report not found: {path}')
    with path.open(encoding='utf-8') as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError(f'{path} is not a JSON object')
    return data


def _rms(values: Sequence[float]) -> float:
    vals = [float(v) for v in values]
    if not vals:
        return 0.0
    return math.sqrt(sum(v * v for v in vals) / len(vals))


def _mean(values: Sequence[float]) -> Optional[float]:
    vals = [float(v) for v in values]
    if not vals:
        return None
    return statistics.fmean(vals)


def _round_cov(v: float) -> float:
    if v >= UNUSED * 0.5:
        return UNUSED
    if v == 0.0:
        return 0.0
    mag = 10 ** math.floor(math.log10(abs(v)))
    return round(v / mag, 2) * mag


def _fmt(v: float) -> str:
    if v >= UNUSED * 0.5:
        return '1e6'
    if v == 0.0:
        return '0.0'
    if abs(v) >= 0.01:
        s = f'{v:.3g}'
        return s if '.' in s else f'{s}.0'
    # 9.9e-4 style (matches existing ekf yaml)
    return f'{v:.1e}'.replace('e-0', 'e-')


def _yaml_diag(diag: Sequence[float], indent: str = '      ') -> str:
    n = 15
    if len(diag) != n:
        raise ValueError(f'diagonal must have {n} entries, got {len(diag)}')
    rows = []
    for i in range(n):
        cells = [_fmt(diag[i] if i == j else 0.0) for j in range(n)]
        line = ', '.join(f'{c:<5}' for c in cells).rstrip()
        if i == 0:
            rows.append(f'{indent}[{line}')
        else:
            rows.append(f'{indent} {line}')
    return ',\n'.join(rows) + ']'


def _odom_stats(odom: Dict[str, Any]) -> Dict[str, Any]:
    trials = odom.get('trials')
    if not isinstance(trials, list) or not trials:
        raise ValueError('odom report has no trials[]')

    fwd_err: List[float] = []
    strafe_err: List[float] = []
    vx_err: List[float] = []
    vy_err: List[float] = []
    yaw_abs: List[float] = []
    scales_fwd: List[float] = []
    scales_strafe: List[float] = []

    for t in trials:
        if not isinstance(t, dict):
            continue
        motion = str(t.get('motion', ''))
        duration = float(t.get('duration_s') or 0.0)
        real = t.get('real_measured_m')
        if real is None:
            continue
        real_m = float(real)
        if 'forward' in motion:
            odom_m = float(t.get('odom_forward_m') or 0.0)
            err = real_m - odom_m
            fwd_err.append(err)
            if duration > 1e-6:
                vx_err.append(err / duration)
            sc = t.get('scale_real_over_odom')
            if sc is not None:
                scales_fwd.append(float(sc))
        elif 'strafe' in motion:
            odom_m = abs(float(t.get('odom_lateral_m') or 0.0))
            err = real_m - odom_m
            strafe_err.append(err)
            if duration > 1e-6:
                vy_err.append(err / duration)
            sc = t.get('scale_real_over_odom')
            if sc is not None:
                scales_strafe.append(float(sc))
        yaw_abs.append(abs(float(t.get('imu_yaw_delta_deg') or 0.0)))

    summary = odom.get('summary') or {}
    return {
        'n_forward': len(fwd_err),
        'n_strafe': len(strafe_err),
        'fwd_rmse_m': _rms(fwd_err),
        'strafe_rmse_m': _rms(strafe_err) if strafe_err else None,
        'vx_rmse': _rms(vx_err) if vx_err else None,
        'vy_rmse': _rms(vy_err) if vy_err else None,
        'scale_fwd': _mean(scales_fwd) or summary.get('forward_scale_real_over_odom'),
        'scale_strafe': _mean(scales_strafe) or summary.get(
            'strafe_combined_scale_real_over_odom'),
        'yaw_during_move_max_deg': max(yaw_abs) if yaw_abs else 0.0,
        'fwd_err': fwd_err,
        'strafe_err': strafe_err,
    }


def _imu_stats(imu: Dict[str, Any]) -> Dict[str, Any]:
    static = imu.get('static') or {}
    rotations = imu.get('rotations') or []
    good_scales: List[float] = []
    skipped = 0
    for r in rotations:
        if not isinstance(r, dict):
            continue
        sc = r.get('scale_real_over_imu')
        if sc is None:
            continue
        scf = float(sc)
        # Drop wrap-around trials (e.g. +180 reporting −42°).
        if scf <= 0.0 or scf > 2.0 or scf < 0.5:
            skipped += 1
            continue
        good_scales.append(scf)

    wz_std = float(static.get('wz_std') or 0.0)
    ax_std = float(static.get('ax_std') or 0.0)
    ay_std = float(static.get('ay_std') or 0.0)
    return {
        'drift_deg_per_min': float(static.get('yaw_drift_deg_per_min') or 0.0),
        'wz_std': wz_std,
        'ax_std': ax_std,
        'ay_std': ay_std,
        'yaw_scale': _mean(good_scales),
        'n_rot_used': len(good_scales),
        'n_rot_skipped': skipped,
        'fuse_accel': ax_std < 0.05 and ay_std < 0.05,
    }


def _q_pose(rmse: Optional[float]) -> float:
    if rmse is None:
        return POSE_R_FLOOR
    return _round_cov(max(rmse * rmse, POSE_R_FLOOR))


def _build_diag(
    q_x: float,
    q_y: float,
    q_vx: float,
    q_vy: Optional[float],
    p_mode: bool,
) -> List[float]:
    """p_mode True → initial P (velocity P is VEL_P); False → process Q."""
    vy = UNUSED if q_vy is None else (VEL_P if p_mode else q_vy)
    vx = VEL_P if p_mode else q_vx
    yaw = YAW_P if p_mode else YAW_Q
    # Match existing xdrive layout: P_vpitch kept small; Q_vpitch unused.
    vpitch = VEL_P if p_mode else UNUSED
    d = [UNUSED] * 15
    d[0] = q_x
    d[1] = q_y
    d[5] = yaw
    d[6] = vx
    d[7] = vy
    d[10] = vpitch
    d[11] = VYW_P if p_mode else VYW_Q
    return [_round_cov(v) for v in d]


def _analyze(
    robot: str,
    odom_s: Dict[str, Any],
    imu_s: Dict[str, Any],
    holonomic: bool,
) -> Tuple[str, Dict[str, Any]]:
    q_x = _q_pose(odom_s['fwd_rmse_m'])
    q_y = _q_pose(odom_s['strafe_rmse_m'] if holonomic else odom_s['fwd_rmse_m'])
    q_vx = VX_Q_FLOOR
    q_vy_h = VX_Q_FLOOR if holonomic else None

    notes = []
    sf = odom_s['scale_fwd']
    if sf is not None:
        notes.append(
            f"Forward scale real/odom = {float(sf):.4f} "
            f"({'odom low' if float(sf) > 1.01 else 'odom high' if float(sf) < 0.99 else '≈1'})."
        )
    notes.append(
        f"Forward position RMSE = {odom_s['fwd_rmse_m']:.4f} m → "
        f"Q_x = max(RMSE², {POSE_R_FLOOR:g}) = {q_x:g}."
    )
    if odom_s['strafe_rmse_m'] is not None:
        notes.append(
            f"Strafe position RMSE = {odom_s['strafe_rmse_m']:.4f} m → "
            f"Q_y = {q_y:g}."
        )
    else:
        notes.append(
            f"No strafe trials — Q_y uses forward RMSE floor → {q_y:g}."
        )
    notes.append(
        f"Q_vx = {q_vx:g} (process floor so EKF can follow speed changes; "
        f"vx RMSE from tape is much smaller)."
    )
    if imu_s['yaw_scale'] is not None:
        notes.append(
            f"IMU yaw scale (good rotations only) = {imu_s['yaw_scale']:.4f} "
            f"({imu_s['n_rot_used']} used, {imu_s['n_rot_skipped']} skipped as wrap)."
        )
    notes.append(
        f"IMU static drift {imu_s['drift_deg_per_min']:.3f} deg/min, "
        f"wz σ={imu_s['wz_std']:.4f} rad/s → keep Q_yaw={YAW_Q:g}, Q_vyaw={VYW_Q:g}."
    )
    if imu_s['fuse_accel']:
        notes.append('ax/ay static σ is low — accel fusion could be considered.')
    else:
        notes.append(
            f"ax σ={imu_s['ax_std']:.3f}, ay σ={imu_s['ay_std']:.3f} m/s² → "
            'do NOT fuse ax/ay (same as current EKF).'
        )
    notes.append(
        f"Max |IMU yaw| during wheel runs = {odom_s['yaw_during_move_max_deg']:.2f} deg "
        '(wheel yaw stays OFF; IMU owns heading).'
    )
    if holonomic:
        notes.append(
            f'{robot} is holonomic: full EKF fuses vy; _diff sets vy P/Q to 1e6.'
        )
    else:
        notes.append(f'{robot} is skid-steer: single EKF, vy unused (1e6).')

    variants: Dict[str, Any] = {}
    if holonomic:
        variants['full'] = {
            'name': f'ekf_{robot}',
            'p': _build_diag(q_x, q_y, q_vx, q_vy_h, True),
            'q': _build_diag(q_x, q_y, q_vx, q_vy_h, False),
        }
        variants['diff'] = {
            'name': f'ekf_{robot}_diff',
            'p': _build_diag(q_x, q_y, q_vx, None, True),
            'q': _build_diag(q_x, q_y, q_vx, None, False),
        }
    else:
        variants['diff'] = {
            'name': f'ekf_{robot}',
            'p': _build_diag(q_x, q_y, q_vx, None, True),
            'q': _build_diag(q_x, q_y, q_vx, None, False),
        }

    analysis = '\n'.join(f'- {n}' for n in notes)
    meta = {
        'q_x': q_x, 'q_y': q_y, 'q_vx': q_vx, 'q_vy': q_vy_h,
        'notes': notes,
    }
    return analysis, {**variants, 'meta': meta}


def _md_variant(block: Dict[str, Any]) -> str:
    p_named = ', '.join(
        f'{STATE[i]}={_fmt(block["p"][i])}' for i in range(15)
        if block['p'][i] < UNUSED * 0.5
    )
    q_named = ', '.join(
        f'{STATE[i]}={_fmt(block["q"][i])}' for i in range(15)
        if block['q'][i] < UNUSED * 0.5
    )
    return '\n'.join([
        f'### `{block["name"]}.yaml`',
        '',
        f'P (used): {p_named}',
        f'Q (used): {q_named}',
        '',
        '```yaml',
        '    initial_estimate_covariance:',
        _yaml_diag(block['p']),
        '',
        '    process_noise_covariance:',
        _yaml_diag(block['q']),
        '```',
        '',
    ])


def main() -> int:
    try:
        odom = _load(ODOM_REPORT)
        imu = _load(IMU_REPORT)
        robot = str(odom.get('robot') or '').strip().lower()
        if robot not in HOLONOMIC and robot not in DIFF_ONLY:
            raise ValueError(
                f"Unknown robot '{robot}' in {ODOM_REPORT.name}. "
                f'Expected one of {sorted(HOLONOMIC | DIFF_ONLY)}.'
            )
        odom_s = _odom_stats(odom)
        imu_s = _imu_stats(imu)
        holonomic = robot in HOLONOMIC
        analysis, variants = _analyze(robot, odom_s, imu_s, holonomic)

        out_dir = REPORTS / f'ekf_{robot}'
        out_dir.mkdir(parents=True, exist_ok=True)
        md_path = out_dir / 'ekf_covariance.md'
        json_path = out_dir / 'ekf_covariance.json'

        blocks_md = []
        json_blocks = {}
        for key in ('full', 'diff'):
            if key not in variants:
                continue
            b = variants[key]
            blocks_md.append(_md_variant(b))
            json_blocks[b['name']] = {
                'initial_estimate_covariance_diag': b['p'],
                'process_noise_covariance_diag': b['q'],
                'state_order': list(STATE),
            }

        md = '\n'.join([
            f'# EKF covariance — {robot}',
            f'Generated: {datetime.now().isoformat(timespec="seconds")}',
            '',
            '## Inputs',
            f'- odom: `{ODOM_REPORT.name}`',
            f'- imu:  `{IMU_REPORT.name}` (robot field: {imu.get("robot")})',
            '',
            '## Analysis',
            analysis,
            '',
            '## Copy into the EKF yaml',
            'Unused axes are `1e6`. Diagonals only.',
            '',
            *blocks_md,
        ])
        md_path.write_text(md, encoding='utf-8')
        json_path.write_text(
            json.dumps({
                'robot': robot,
                'generated': datetime.now().isoformat(timespec='seconds'),
                'odom_report': str(ODOM_REPORT),
                'imu_report': str(IMU_REPORT),
                'analysis': variants['meta']['notes'],
                'variants': json_blocks,
            }, indent=2),
            encoding='utf-8',
        )
        print(md)
        print(f'Saved {md_path}')
        print(f'Saved {json_path}')
        return 0
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
