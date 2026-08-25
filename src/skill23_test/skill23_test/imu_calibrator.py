#!/usr/bin/env python3
"""Interactive IMU calibration for Skill23 robots.

Tests raw /imu only (launch imu_test.launch.py — no EKF).
/cmd_vel angular.z is **Titan duty** (−1…1), NOT rad/s.

Workflow:
  1. Static test — robot still, records yaw drift + ax/ay bias.
  2. Rotation tests — open-loop spin at fixed duty; you enter real angle.
  3. Report saved to skill23_test/reports/ for later EKF tuning.

Run wheel odom test first (odom_test.launch.py + odom_calibrator), then this.
"""

from __future__ import annotations

import json
import math
import statistics
import sys
import threading
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu

from skill23_test.report_paths import get_reports_dir


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def _delta_yaw_deg(start_rad: float, end_rad: float) -> float:
    dyaw = end_rad - start_rad
    while dyaw > math.pi:
        dyaw -= 2.0 * math.pi
    while dyaw < -math.pi:
        dyaw += 2.0 * math.pi
    return math.degrees(dyaw)


@dataclass
class ImuReading:
    yaw_rad: float = 0.0
    wz: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    stamp: float = 0.0
    valid: bool = False


@dataclass
class StaticTrial:
    duration_s: float
    sample_count: int
    yaw_start_deg: float
    yaw_end_deg: float
    yaw_drift_deg: float
    yaw_drift_deg_per_min: float
    wz_mean: float
    wz_std: float
    ax_mean: float
    ax_std: float
    ay_mean: float
    ay_std: float
    az_mean: float
    az_std: float


@dataclass
class RotateTrial:
    commanded_deg: float
    rotate_duty: float
    rotate_duration_s_per_deg: float
    duration_s: float
    imu_yaw_delta_deg: float
    wz_mean: float
    ax_mean: float
    ay_mean: float
    imu_start: dict = field(default_factory=dict)
    imu_end: dict = field(default_factory=dict)
    real_measured_deg: Optional[float] = None
    scale_real_over_imu: Optional[float] = None


ROBOT_EKF_CONFIG = {
    'grip': 'grip/config/ekf_grip.yaml',
    'mecanum': 'mecanum/config/ekf_mecanum.yaml',
    'xdrive': 'xdrive/config/ekf_xdrive.yaml',
    'allterrain': 'allterrain/config/ekf_allterrain.yaml',
}


class ImuCalibrator(Node):
    def __init__(self) -> None:
        super().__init__('imu_calibrator')
        self.declare_parameter('robot', 'grip')
        self.declare_parameter('rotate_duty', 0.25)
        self.declare_parameter('rotate_duration_s_per_deg', 0.05)
        self.declare_parameter('static_duration_s', 10.0)
        self.declare_parameter('cmd_rate_hz', 20.0)
        self.declare_parameter('settle_s', 0.5)
        self.declare_parameter('max_rotate_duration_s', 30.0)
        self.declare_parameter('rotate_angles_deg', [90.0, -90.0, 180.0])

        self.robot = str(self.get_parameter('robot').value)
        self.rotate_duty = float(self.get_parameter('rotate_duty').value)
        self.rotate_duration_s_per_deg = float(
            self.get_parameter('rotate_duration_s_per_deg').value)
        self.static_duration_s = float(self.get_parameter('static_duration_s').value)
        self.cmd_rate_hz = float(self.get_parameter('cmd_rate_hz').value)
        self.settle_s = float(self.get_parameter('settle_s').value)
        self.max_rotate_duration_s = float(self.get_parameter('max_rotate_duration_s').value)
        self.rotate_angles = [float(a) for a in self.get_parameter('rotate_angles_deg').value]

        self._lock = threading.Lock()
        self._imu = ImuReading()

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Imu, '/imu', self._on_imu, 50)

        self._reports_dir = get_reports_dir()
        self.get_logger().info(f'Reports directory: {self._reports_dir}')

    def _on_imu(self, msg: Imu) -> None:
        q = msg.orientation
        with self._lock:
            self._imu = ImuReading(
                yaw_rad=_yaw_from_quat(q.x, q.y, q.z, q.w),
                wz=msg.angular_velocity.z,
                ax=msg.linear_acceleration.x,
                ay=msg.linear_acceleration.y,
                az=msg.linear_acceleration.z,
                stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                valid=True,
            )

    def _snapshot(self) -> ImuReading:
        with self._lock:
            return ImuReading(**asdict(self._imu))

    def _stop(self) -> None:
        self._cmd_pub.publish(Twist())

    def _wait_settle(self) -> None:
        self._stop()
        time.sleep(self.settle_s)

    def _collect_imu_series(self, duration_s: float) -> List[ImuReading]:
        samples: List[ImuReading] = []
        t0 = time.monotonic()
        while time.monotonic() - t0 < duration_s:
            snap = self._snapshot()
            if snap.valid:
                samples.append(snap)
            time.sleep(0.02)
        return samples

    @staticmethod
    def _series_stats(values: List[float]) -> tuple[float, float]:
        if not values:
            return 0.0, 0.0
        if len(values) == 1:
            return values[0], 0.0
        return statistics.mean(values), statistics.pstdev(values)

    def _run_static(self) -> StaticTrial:
        self._wait_settle()
        if not self._snapshot().valid:
            raise RuntimeError('/imu not received — is control_server running?')

        print(f'  Recording static IMU for {self.static_duration_s:.1f} s — keep robot still...')
        samples = self._collect_imu_series(self.static_duration_s)
        if len(samples) < 2:
            raise RuntimeError('Not enough IMU samples during static test')

        yaw_start = math.degrees(samples[0].yaw_rad)
        yaw_end = math.degrees(samples[-1].yaw_rad)
        drift = _delta_yaw_deg(samples[0].yaw_rad, samples[-1].yaw_rad)
        drift_per_min = drift * (60.0 / max(self.static_duration_s, 0.1))

        wz_m, wz_s = self._series_stats([s.wz for s in samples])
        ax_m, ax_s = self._series_stats([s.ax for s in samples])
        ay_m, ay_s = self._series_stats([s.ay for s in samples])
        az_m, az_s = self._series_stats([s.az for s in samples])

        return StaticTrial(
            duration_s=self.static_duration_s,
            sample_count=len(samples),
            yaw_start_deg=yaw_start,
            yaw_end_deg=yaw_end,
            yaw_drift_deg=drift,
            yaw_drift_deg_per_min=drift_per_min,
            wz_mean=wz_m,
            wz_std=wz_s,
            ax_mean=ax_m,
            ax_std=ax_s,
            ay_mean=ay_m,
            ay_std=ay_s,
            az_mean=az_m,
            az_std=az_s,
        )

    def _open_loop_rotate(self, target_deg: float) -> RotateTrial:
        if target_deg == 0.0:
            raise ValueError('rotation angle must be non-zero')
        if self.rotate_duration_s_per_deg <= 0.0:
            raise ValueError('rotate_duration_s_per_deg must be positive')

        duty = self.rotate_duty if target_deg > 0.0 else -self.rotate_duty
        duration = min(
            abs(target_deg) * self.rotate_duration_s_per_deg,
            self.max_rotate_duration_s,
        )

        self._wait_settle()
        start = self._snapshot()
        if not start.valid:
            raise RuntimeError('/imu not received')

        period = 1.0 / max(1.0, self.cmd_rate_hz)
        t0 = time.monotonic()
        samples: List[ImuReading] = []
        self.get_logger().info(
            f'rotate: duty={duty:.2f} (NOT rad/s) target={target_deg:+.1f} deg '
            f'time={duration:.2f} s')

        while True:
            elapsed = time.monotonic() - t0
            if elapsed >= duration:
                break
            msg = Twist()
            msg.angular.z = float(duty)
            self._cmd_pub.publish(msg)
            snap = self._snapshot()
            if snap.valid:
                samples.append(snap)
            time.sleep(period)

        self._stop()
        time.sleep(0.15)
        end = self._snapshot()

        imu_delta = _delta_yaw_deg(start.yaw_rad, end.yaw_rad)
        wz_m, _ = self._series_stats([s.wz for s in samples])
        ax_m, _ = self._series_stats([s.ax for s in samples])
        ay_m, _ = self._series_stats([s.ay for s in samples])

        return RotateTrial(
            commanded_deg=target_deg,
            rotate_duty=abs(self.rotate_duty),
            rotate_duration_s_per_deg=self.rotate_duration_s_per_deg,
            duration_s=duration,
            imu_yaw_delta_deg=imu_delta,
            wz_mean=wz_m,
            ax_mean=ax_m,
            ay_mean=ay_m,
            imu_start={
                'yaw_deg': math.degrees(start.yaw_rad),
                'wz': start.wz,
                'ax': start.ax,
                'ay': start.ay,
            },
            imu_end={
                'yaw_deg': math.degrees(end.yaw_rad),
                'wz': end.wz,
                'ax': end.ax,
                'ay': end.ay,
            },
        )

    def _prompt_real_rotate(self, trial: RotateTrial) -> None:
        print()
        print(f'  IMU yaw change: {trial.imu_yaw_delta_deg:+.2f} deg')
        print(f'  Mean wz during move: {trial.wz_mean:+.4f} rad/s')
        print(f'  Mean ax/ay during move: {trial.ax_mean:+.4f} / {trial.ay_mean:+.4f} m/s²')
        direction = 'CCW (+)' if trial.commanded_deg > 0 else 'CW (−)'
        while True:
            raw = input(
                f'  Enter REAL angle turned in deg {direction}, positive number: '
            ).strip()
            try:
                val = float(raw)
                if val < 0.0:
                    print('  Use positive magnitude along the turn direction.')
                    continue
                signed = val if trial.commanded_deg > 0 else -val
                trial.real_measured_deg = signed
                if abs(trial.imu_yaw_delta_deg) > 1e-3:
                    trial.scale_real_over_imu = signed / trial.imu_yaw_delta_deg
                else:
                    trial.scale_real_over_imu = None
                break
            except ValueError:
                print('  Invalid number, try again.')

    def _run_interactive(
        self,
    ) -> tuple[StaticTrial, List[RotateTrial]]:
        print('=' * 60)
        print(f'  IMU calibration — robot: {self.robot}')
        print(f'  /cmd_vel.angular.z duty (−1…1, NOT rad/s): {self.rotate_duty}')
        print(f'  Stop timer = |deg| × {self.rotate_duration_s_per_deg} s/deg')
        print('  Raw /imu only — no EKF during this test.')
        print('=' * 60)

        print()
        print('--- STATIC (robot must not move) ---')
        input('Press Enter to start static IMU capture...')
        static = self._run_static()
        print()
        print(f'  Yaw drift: {static.yaw_drift_deg:+.3f} deg '
              f'({static.yaw_drift_deg_per_min:+.3f} deg/min)')
        print(f'  ax mean±std: {static.ax_mean:+.4f} ± {static.ax_std:.4f} m/s²')
        print(f'  ay mean±std: {static.ay_mean:+.4f} ± {static.ay_std:.4f} m/s²')

        rotates: List[RotateTrial] = []
        for angle in self.rotate_angles:
            label = 'CCW' if angle > 0 else 'CW'
            print()
            print(f'--- ROTATE {label} {abs(angle):.0f} deg ---')
            input('Press Enter to start rotation (Ctrl+C aborts)...')
            trial = self._open_loop_rotate(angle)
            self._prompt_real_rotate(trial)
            rotates.append(trial)

        return static, rotates

    @staticmethod
    def _mean_scale(rotates: List[RotateTrial]) -> Optional[float]:
        scales = [r.scale_real_over_imu for r in rotates if r.scale_real_over_imu is not None]
        if not scales:
            return None
        return sum(scales) / len(scales)

    def _write_report(self, static: StaticTrial, rotates: List[RotateTrial]) -> Path:
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        base = f'{self.robot}_imu_cal_{ts}'
        md_path = self._reports_dir / f'{base}.md'
        json_path = self._reports_dir / f'{base}.json'

        rot_scale = self._mean_scale(rotates)
        ekf_cfg = ROBOT_EKF_CONFIG.get(self.robot, f'{self.robot}/config/ekf_{self.robot}.yaml')

        lines = [
            f'# IMU calibration — {self.robot}',
            f'Generated: {datetime.now().isoformat(timespec="seconds")}',
            '',
            '## Test settings',
            f'- rotate_duty (cmd_vel.angular.z): {self.rotate_duty}',
            f'- rotate_duration_s_per_deg (timer only): {self.rotate_duration_s_per_deg}',
            f'- static_duration_s: {self.static_duration_s}',
            f'- **/cmd_vel is Titan duty (−1…1), NOT rad/s**',
            '',
            '## Static summary',
            f'- Yaw drift: **{static.yaw_drift_deg:+.3f} deg** '
            f'({static.yaw_drift_deg_per_min:+.3f} deg/min over {static.duration_s:.0f} s)',
            f'- ax bias (mean): {static.ax_mean:+.4f} m/s² (σ={static.ax_std:.4f})',
            f'- ay bias (mean): {static.ay_mean:+.4f} m/s² (σ={static.ay_std:.4f})',
            f'- wz noise (mean): {static.wz_mean:+.4f} rad/s (σ={static.wz_std:.4f})',
            '',
            '## Rotation summary',
        ]
        if rot_scale is not None:
            lines.append(
                f'- **Yaw scale** (real/IMU): **{rot_scale:.4f}** '
                f'(IMU reads high → lower yaw trust in EKF)')
        lines.append('')
        lines.append('## EKF tuning (`' + ekf_cfg + '`)')
        lines.append('- Fuse IMU: yaw, vyaw, ax, ay (after wheel odom is scaled).')
        lines.append('- Static ax/ay → adjust `linear_acceleration_covariance` in imu_component.')
        lines.append('- Yaw drift / rotation scale → adjust orientation + angular_velocity cov.')
        lines.append('- Compare with wheel odom report from `odom_calibrator` before final EKF Q/R.')
        lines.append('')
        lines.append('## Static trial')
        lines.append('')
        lines.append('| duration(s) | samples | yaw_drift(deg) | drift(deg/min) | ax_mean | ay_mean |')
        lines.append('|-------------|---------|----------------|----------------|---------|---------|')
        lines.append(
            f'| {static.duration_s:.1f} | {static.sample_count} | '
            f'{static.yaw_drift_deg:+.3f} | {static.yaw_drift_deg_per_min:+.3f} | '
            f'{static.ax_mean:+.4f} | {static.ay_mean:+.4f} |')
        lines.append('')
        lines.append('## Rotation trials')
        lines.append('')
        lines.append(
            '| cmd(deg) | duty | time(s) | imu_yaw(deg) | real(deg) | scale | ax_mean | ay_mean |')
        lines.append(
            '|----------|------|---------|--------------|-----------|-------|---------|---------|')
        for r in rotates:
            real_s = f'{r.real_measured_deg:+.2f}' if r.real_measured_deg is not None else '—'
            scale_s = f'{r.scale_real_over_imu:.4f}' if r.scale_real_over_imu else '—'
            lines.append(
                f'| {r.commanded_deg:+.0f} | {r.rotate_duty:.2f} | {r.duration_s:.2f} | '
                f'{r.imu_yaw_delta_deg:+.2f} | {real_s} | {scale_s} | '
                f'{r.ax_mean:+.4f} | {r.ay_mean:+.4f} |')

        md_path.write_text('\n'.join(lines) + '\n', encoding='utf-8')
        payload = {
            'robot': self.robot,
            'generated': datetime.now().isoformat(timespec='seconds'),
            'settings': {
                'rotate_duty': self.rotate_duty,
                'rotate_duration_s_per_deg': self.rotate_duration_s_per_deg,
                'static_duration_s': self.static_duration_s,
                'cmd_vel_is_duty_not_rad_s': True,
            },
            'summary': {
                'static_yaw_drift_deg_per_min': static.yaw_drift_deg_per_min,
                'static_ax_mean': static.ax_mean,
                'static_ay_mean': static.ay_mean,
                'rotation_yaw_scale_real_over_imu': rot_scale,
            },
            'static': asdict(static),
            'rotations': [asdict(r) for r in rotates],
        }
        json_path.write_text(json.dumps(payload, indent=2), encoding='utf-8')
        return md_path

    def run(self) -> None:
        try:
            print('Waiting for /imu...')
            for _ in range(100):
                if self._imu.valid:
                    break
                time.sleep(0.05)
            if not self._imu.valid:
                print('ERROR: No /imu. Launch imu_test.launch.py first.', file=sys.stderr)
                return
            static, rotates = self._run_interactive()
            path = self._write_report(static, rotates)
            print()
            print('=' * 60)
            print(f'  Report saved: {path}')
            print('  Next: tune EKF using this + the odom_cal report.')
            print('=' * 60)
        except KeyboardInterrupt:
            print('\nInterrupted — stopping motors.')
        finally:
            self._stop()


def main() -> None:
    rclpy.init()
    node = ImuCalibrator()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        node.run()
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
