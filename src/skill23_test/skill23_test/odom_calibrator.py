#!/usr/bin/env python3
"""Interactive open-loop odometry calibration for Skill23 robots.

/cmd_vel linear.x and linear.y are **Titan duty cycle** (−1…1), NOT m/s.
This script publishes a fixed duty (linear_duty / strafe_duty) with no PID
and no rotation correction.

Stop time = commanded_distance × move_duration_s_per_m (timer only — not sent
to the motors).

Workflow (terminal prompts):
  1. Press Enter to start each commanded distance.
  2. Robot moves straight or strafe at fixed duty.
  3. Script records /wheel/odom (and optional /odometry/filtered, /imu).
  4. You enter the real measured distance (tape measure).
  5. After all trials, writes a report under skill23_test/reports/.

Run imu_calibrator separately after this (see imu_test.launch.py).

Usage (after odom_test.launch.py is running):
  ros2 run skill23_test odom_calibrator --ros-args -p robot:=xdrive
"""

from __future__ import annotations

import json
import math
import sys
import threading
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu

from skill23_test.report_paths import get_reports_dir


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def _project_forward(dx: float, dy: float, yaw: float) -> float:
    return dx * math.cos(yaw) + dy * math.sin(yaw)


def _project_lateral(dx: float, dy: float, yaw: float) -> float:
    return -dx * math.sin(yaw) + dy * math.cos(yaw)


@dataclass
class OdomSample:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    stamp: float = 0.0
    valid: bool = False


@dataclass
class TrialRecord:
    motion: str
    commanded_m: float
    duty: float
    move_duration_s_per_m: float
    duration_s: float
    odom_forward_m: float
    odom_lateral_m: float
    odom_total_m: float
    imu_yaw_delta_deg: float
    wheel_odom_start: dict = field(default_factory=dict)
    wheel_odom_end: dict = field(default_factory=dict)
    real_measured_m: Optional[float] = None
    scale_real_over_odom: Optional[float] = None


ROBOT_EKF_CONFIG = {
    'grip': 'grip/config/ekf_grip.yaml',
    'mecanum': 'mecanum/config/ekf_mecanum.yaml',
    'xdrive': 'xdrive/config/ekf_xdrive.yaml',
    'allterrain': 'allterrain/config/ekf_allterrain.yaml',
}


class OdomCalibrator(Node):
    def __init__(self) -> None:
        super().__init__('odom_calibrator')
        self.declare_parameter('robot', 'grip')
        self.declare_parameter('linear_duty', 0.25)
        self.declare_parameter('strafe_duty', 0.25)
        self.declare_parameter('move_duration_s_per_m', 5.0)
        self.declare_parameter('cmd_rate_hz', 20.0)
        self.declare_parameter('settle_s', 0.5)
        self.declare_parameter('max_duration_s', 30.0)
        self.declare_parameter('use_filtered_odom', False)
        self.declare_parameter(
            'forward_distances_m', [0.5, 1.0, 2.0])
        self.declare_parameter('strafe_distances_m', [0.5, 1.0])

        self.robot = str(self.get_parameter('robot').value)
        self.linear_duty = float(self.get_parameter('linear_duty').value)
        self.strafe_duty = float(self.get_parameter('strafe_duty').value)
        self.move_duration_s_per_m = float(self.get_parameter('move_duration_s_per_m').value)
        self.cmd_rate_hz = float(self.get_parameter('cmd_rate_hz').value)
        self.settle_s = float(self.get_parameter('settle_s').value)
        self.max_duration_s = float(self.get_parameter('max_duration_s').value)
        self.use_filtered = bool(self.get_parameter('use_filtered_odom').value)
        self.forward_distances = list(self.get_parameter('forward_distances_m').value)
        self.strafe_distances = list(self.get_parameter('strafe_distances_m').value)

        self._lock = threading.Lock()
        self._wheel = OdomSample()
        self._filtered = OdomSample()
        self._imu_yaw: Optional[float] = None

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/wheel/odom', self._on_wheel, 10)
        if self.use_filtered:
            self.create_subscription(
                Odometry, '/odometry/filtered', self._on_filtered, 10)
        self.create_subscription(Imu, '/imu', self._on_imu, 10)

        self._reports_dir = get_reports_dir()
        self.get_logger().info(f'Reports directory: {self._reports_dir}')

    def _on_wheel(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        with self._lock:
            self._wheel = OdomSample(
                x=msg.pose.pose.position.x,
                y=msg.pose.pose.position.y,
                yaw=_yaw_from_quat(q.x, q.y, q.z, q.w),
                stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                valid=True,
            )

    def _on_filtered(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        with self._lock:
            self._filtered = OdomSample(
                x=msg.pose.pose.position.x,
                y=msg.pose.pose.position.y,
                yaw=_yaw_from_quat(q.x, q.y, q.z, q.w),
                stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                valid=True,
            )

    def _on_imu(self, msg: Imu) -> None:
        q = msg.orientation
        with self._lock:
            self._imu_yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)

    def _snapshot(self) -> Tuple[OdomSample, Optional[float]]:
        with self._lock:
            return OdomSample(**asdict(self._wheel)), self._imu_yaw

    def _stop(self) -> None:
        self._cmd_pub.publish(Twist())

    def _publish_cmd(self, duty_x: float, duty_y: float) -> None:
        """Publish Titan duty on /cmd_vel (NOT linear velocity in m/s)."""
        msg = Twist()
        msg.linear.x = float(duty_x)
        msg.linear.y = float(duty_y)
        msg.angular.z = 0.0
        self._cmd_pub.publish(msg)

    def _wait_settle(self) -> None:
        self._stop()
        time.sleep(self.settle_s)

    def _open_loop_move(
        self,
        motion: str,
        target_m: float,
        duty: float,
    ) -> TrialRecord:
        if target_m <= 0.0:
            raise ValueError('target distance must be positive')
        if self.move_duration_s_per_m <= 0.0:
            raise ValueError('move_duration_s_per_m must be positive')

        duration = min(target_m * self.move_duration_s_per_m, self.max_duration_s)
        duty_x = duty if motion == 'forward' else 0.0
        duty_y = duty if motion == 'strafe_left' else 0.0
        if motion == 'strafe_right':
            duty_y = -duty

        self._wait_settle()
        start_odom, start_imu = self._snapshot()
        if not start_odom.valid:
            raise RuntimeError('/wheel/odom not received — is the drive node running?')

        start_yaw = start_odom.yaw
        period = 1.0 / max(1.0, self.cmd_rate_hz)
        t0 = time.monotonic()
        self.get_logger().info(
            f'{motion}: duty={duty:.2f} (NOT m/s) target={target_m:.2f} m '
            f'time={duration:.2f} s (open loop, no rotation fix)')

        while True:
            elapsed = time.monotonic() - t0
            if elapsed >= duration:
                break
            self._publish_cmd(duty_x, duty_y)
            time.sleep(period)

        self._stop()
        time.sleep(0.15)
        end_odom, end_imu = self._snapshot()

        dx = end_odom.x - start_odom.x
        dy = end_odom.y - start_odom.y
        fwd = _project_forward(dx, dy, start_yaw)
        lat = _project_lateral(dx, dy, start_yaw)

        yaw_delta_deg = 0.0
        if start_imu is not None and end_imu is not None:
            dyaw = end_imu - start_imu
            while dyaw > math.pi:
                dyaw -= 2.0 * math.pi
            while dyaw < -math.pi:
                dyaw += 2.0 * math.pi
            yaw_delta_deg = math.degrees(dyaw)

        return TrialRecord(
            motion=motion,
            commanded_m=target_m,
            duty=duty,
            move_duration_s_per_m=self.move_duration_s_per_m,
            duration_s=duration,
            odom_forward_m=fwd,
            odom_lateral_m=lat,
            odom_total_m=math.hypot(dx, dy),
            imu_yaw_delta_deg=yaw_delta_deg,
            wheel_odom_start={'x': start_odom.x, 'y': start_odom.y, 'yaw_deg': math.degrees(start_yaw)},
            wheel_odom_end={'x': end_odom.x, 'y': end_odom.y, 'yaw_deg': math.degrees(end_odom.yaw)},
        )

    def _supports_strafe(self) -> bool:
        return self.robot in ('mecanum', 'xdrive')

    def _prompt_real(self, trial: TrialRecord) -> None:
        axis = 'forward' if 'forward' in trial.motion else 'lateral'
        odom_val = trial.odom_forward_m if axis == 'forward' else trial.odom_lateral_m
        odom_mag = abs(odom_val) if axis == 'lateral' else odom_val
        dir_label = trial.motion.replace('_', ' ')
        print()
        print(f'  Wheel odom {axis}: {odom_val:+.4f} m  (total path: {trial.odom_total_m:.4f} m)')
        print(f'  IMU yaw change during move: {trial.imu_yaw_delta_deg:+.2f} deg')
        while True:
            raw = input(
                f'  Enter REAL measured distance for {dir_label} (m, positive): '
            ).strip()
            try:
                val = float(raw)
                if val < 0.0:
                    print('  Use positive distance along the move direction.')
                    continue
                trial.real_measured_m = val
                if abs(odom_mag) > 1e-4:
                    trial.scale_real_over_odom = val / odom_mag
                else:
                    trial.scale_real_over_odom = None
                break
            except ValueError:
                print('  Invalid number, try again.')

    def _run_interactive(self) -> List[TrialRecord]:
        records: List[TrialRecord] = []
        print('=' * 60)
        print(f'  Odometry calibration — robot: {self.robot}')
        print(f'  /cmd_vel duty (−1…1, NOT m/s): forward={self.linear_duty}, strafe={self.strafe_duty}')
        print(f'  Stop timer = distance × {self.move_duration_s_per_m} s/m (duty held constant)')
        print('  No PID, no rotation correction. Clear space in front/sides.')
        print('=' * 60)

        for dist in self.forward_distances:
            print()
            print(f'--- FORWARD {dist:.2f} m ---')
            input('Press Enter to start (Ctrl+C aborts and stops motors)...')
            trial = self._open_loop_move('forward', float(dist), self.linear_duty)
            self._prompt_real(trial)
            records.append(trial)

        if self._supports_strafe():
            for dist in self.strafe_distances:
                print()
                print(f'--- STRAFE LEFT {dist:.2f} m ---')
                input('Press Enter to start...')
                trial = self._open_loop_move('strafe_left', float(dist), self.strafe_duty)
                self._prompt_real(trial)
                records.append(trial)
            for dist in self.strafe_distances:
                print()
                print(f'--- STRAFE RIGHT {dist:.2f} m ---')
                input('Press Enter to start...')
                trial = self._open_loop_move('strafe_right', float(dist), self.strafe_duty)
                self._prompt_real(trial)
                records.append(trial)
        else:
            print()
            print(f'  Skipping strafe tests ({self.robot} is skid-steer).')

        return records

    @staticmethod
    def _mean_scale(records: List[TrialRecord], motion_prefix: str) -> Optional[float]:
        scales = [
            r.scale_real_over_odom for r in records
            if r.motion.startswith(motion_prefix) and r.scale_real_over_odom is not None
        ]
        if not scales:
            return None
        return sum(scales) / len(scales)

    def _write_report(self, records: List[TrialRecord]) -> Path:
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        base = f'{self.robot}_odom_cal_{ts}'
        md_path = self._reports_dir / f'{base}.md'
        json_path = self._reports_dir / f'{base}.json'

        fwd_scale = self._mean_scale(records, 'forward')
        strafe_left_scale = self._mean_scale(records, 'strafe_left')
        strafe_right_scale = self._mean_scale(records, 'strafe_right')
        strafe_scale = self._mean_scale(records, 'strafe')

        lines = [
            f'# Odometry calibration — {self.robot}',
            f'Generated: {datetime.now().isoformat(timespec="seconds")}',
            '',
            '## Test settings',
            f'- linear_duty (cmd_vel.linear.x): {self.linear_duty}',
            f'- strafe_duty (cmd_vel.linear.y): {self.strafe_duty}',
            f'- move_duration_s_per_m (timer only): {self.move_duration_s_per_m}',
            f'- **/cmd_vel is Titan duty (−1…1), NOT m/s**',
            '',
            '## Summary',
        ]
        if fwd_scale is not None:
            lines.append(
                f'- **Forward odom scale** (real/odom, multiply odom to get meters): '
                f'**{fwd_scale:.4f}**')
            lines.append(
                f'  → If odom reads high, scale encoders down (~{1.0/fwd_scale:.4f}× ticks).')
        if strafe_left_scale is not None:
            lines.append(
                f'- **Strafe LEFT scale** (real/odom): **{strafe_left_scale:.4f}**')
        if strafe_right_scale is not None:
            lines.append(
                f'- **Strafe RIGHT scale** (real/odom): **{strafe_right_scale:.4f}**')
        if strafe_scale is not None:
            lines.append(
                f'- **Strafe combined scale** (real/odom): **{strafe_scale:.4f}**')
        lines.append('')
        ekf_cfg = ROBOT_EKF_CONFIG.get(self.robot, f'{self.robot}/config/ekf_{self.robot}.yaml')
        lines.append('## EKF notes')
        lines.append(f'- Tune `{ekf_cfg}` from this test (per-robot EKF, not skill23_custom_nav).')
        lines.append('- Compare wheel odom scale to 1.0 before trusting x,y,vx fusion.')
        lines.append('- Large IMU yaw drift during straight runs → check IMU mount / calibration.')
        lines.append('- If stuck wheels still advance odom, reduce EKF trust on wheel x,y,vx.')
        lines.append('')
        lines.append('## Trials')
        lines.append('')
        lines.append(
            '| motion | cmd(m) | duty(−1…1) | time(s) | odom_fwd | odom_lat | real(m) | scale | yaw(deg) |')
        lines.append(
            '|--------|--------|------------|---------|----------|----------|---------|-------|----------|')
        for r in records:
            real_s = f'{r.real_measured_m:.4f}' if r.real_measured_m is not None else '—'
            scale_s = f'{r.scale_real_over_odom:.4f}' if r.scale_real_over_odom else '—'
            lines.append(
                f'| {r.motion} | {r.commanded_m:.2f} | {r.duty:.2f} | '
                f'{r.duration_s:.2f} | {r.odom_forward_m:+.4f} | {r.odom_lateral_m:+.4f} | '
                f'{real_s} | {scale_s} | {r.imu_yaw_delta_deg:+.2f} |')

        md_path.write_text('\n'.join(lines) + '\n', encoding='utf-8')
        payload = {
            'robot': self.robot,
            'generated': datetime.now().isoformat(timespec='seconds'),
            'settings': {
                'linear_duty': self.linear_duty,
                'strafe_duty': self.strafe_duty,
                'move_duration_s_per_m': self.move_duration_s_per_m,
                'cmd_vel_is_duty_not_mps': True,
            },
            'summary': {
                'forward_scale_real_over_odom': fwd_scale,
                'strafe_left_scale_real_over_odom': strafe_left_scale,
                'strafe_right_scale_real_over_odom': strafe_right_scale,
                'strafe_combined_scale_real_over_odom': strafe_scale,
            },
            'trials': [asdict(r) for r in records],
        }
        json_path.write_text(json.dumps(payload, indent=2), encoding='utf-8')
        return md_path

    def run(self) -> None:
        try:
            print('Waiting for /wheel/odom...')
            for _ in range(100):
                if self._wheel.valid:
                    break
                time.sleep(0.05)
            if not self._wheel.valid:
                print('ERROR: No /wheel/odom. Launch odom_test.launch.py first.', file=sys.stderr)
                return
            records = self._run_interactive()
            path = self._write_report(records)
            print()
            print('=' * 60)
            print(f'  Report saved: {path}')
            print('=' * 60)
        except KeyboardInterrupt:
            print('\nInterrupted — stopping motors.')
        finally:
            self._stop()


def main() -> None:
    rclpy.init()
    node = OdomCalibrator()
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
