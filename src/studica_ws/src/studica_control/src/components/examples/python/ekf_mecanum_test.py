#!/usr/bin/env python3
"""Simplified EKF test harness for mecanum — core tests only.

Monitors /odometry/filtered (+ /wheel/odom). Scores PASS/FAIL.

Commands (/ekf_test/cmd std_msgs/String):
  help | status | start <1|2|3|drive> | mark | done
  truth <x> <y> <yaw_deg> | report | reset

Tests:
  1  STATIONARY — still 5+ min, then done
  2  STRAIGHT 2 m — tape measure, then done [optional truth]
  3  SQUARE 2 m — mark each corner, done at start
  drive — free drive (no tape); done → jump/cov summary for log review
"""
from __future__ import annotations

import csv
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger

LOG_DIR = Path('/home/vmx/skill23_ws')
CSV_PATH = LOG_DIR / 'tmp_ekf_mecanum_test.csv'
RESULTS_PATH = LOG_DIR / 'tmp_ekf_mecanum_test_results.txt'


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def deg(rad: float) -> float:
    return math.degrees(rad)


@dataclass
class PoseSample:
    t: float
    x: float
    y: float
    yaw: float
    vx: float = 0.0
    vy: float = 0.0
    vyaw: float = 0.0
    cov_x: float = 0.0
    cov_y: float = 0.0
    cov_yaw: float = 0.0


@dataclass
class TestResult:
    test_id: str
    name: str
    passed: Optional[bool]
    summary: str
    details: Dict[str, float] = field(default_factory=dict)


TEST_HELP = {
    '1': 'STATIONARY — do not touch robot 5+ min, then done',
    '2': 'STRAIGHT 2 m — drive to 2 m mark, then done [optional: truth x y yaw]',
    '3': 'SQUARE 2 m — mark at each corner; done when back at start',
    'drive': 'FREE DRIVE — mix forward/strafe/turn; done when finished',
}


class EkfMecanumTest(Node):
    def __init__(self) -> None:
        super().__init__('ekf_mecanum_test')

        self._t0 = time.monotonic()
        self._filt: Optional[PoseSample] = None
        self._wheel: Optional[PoseSample] = None

        self._active_id: Optional[str] = None
        self._start: Optional[PoseSample] = None
        self._marks: List[PoseSample] = []
        self._truth: Optional[Tuple[float, float, float]] = None
        self._samples: List[PoseSample] = []
        self._results: List[TestResult] = []
        self._cov_trace: List[Tuple[float, float, float, float]] = []

        CSV_PATH.write_text(
            't,source,x,y,yaw_deg,vx,vy,vyaw,cov_x,cov_y,cov_yaw\n'
        )
        RESULTS_PATH.write_text(f'SESSION {time.strftime("%Y-%m-%d %H:%M:%S")}\n')

        self.create_subscription(Odometry, '/odometry/filtered', self._on_filt, 50)
        self.create_subscription(Odometry, '/wheel/odom', self._on_wheel, 50)
        self.create_subscription(String, '/ekf_test/cmd', self._on_cmd, 10)
        self.create_service(Trigger, '/ekf_test/help', self._on_help_srv)
        self.create_timer(1.0, self._heartbeat)

        self.get_logger().info('EKF core test ready (tests 1, 2, 3, drive)')
        self.get_logger().info(f'CSV: {CSV_PATH}')
        self._print_help()

    def _sample_from_odom(self, msg: Odometry) -> PoseSample:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t = msg.twist.twist
        cov = msg.pose.covariance
        return PoseSample(
            t=time.monotonic() - self._t0,
            x=p.x, y=p.y, yaw=yaw_from_quat(q.x, q.y, q.z, q.w),
            vx=t.linear.x, vy=t.linear.y, vyaw=t.angular.z,
            cov_x=cov[0], cov_y=cov[7], cov_yaw=cov[35],
        )

    def _append_csv(self, source: str, s: PoseSample) -> None:
        with CSV_PATH.open('a') as f:
            csv.writer(f).writerow([
                f'{s.t:.3f}', source,
                f'{s.x:.6f}', f'{s.y:.6f}', f'{deg(s.yaw):.4f}',
                f'{s.vx:.6f}', f'{s.vy:.6f}', f'{s.vyaw:.6f}',
                f'{s.cov_x:.6g}', f'{s.cov_y:.6g}', f'{s.cov_yaw:.6g}',
            ])

    def _on_filt(self, msg: Odometry) -> None:
        s = self._sample_from_odom(msg)
        self._filt = s
        self._append_csv('filtered', s)
        if self._active_id is not None:
            self._samples.append(s)
            self._cov_trace.append((s.t, s.cov_x, s.cov_y, s.cov_yaw))

    def _on_wheel(self, msg: Odometry) -> None:
        s = self._sample_from_odom(msg)
        self._wheel = s
        self._append_csv('wheel', s)

    def _heartbeat(self) -> None:
        if self._filt is None:
            self.get_logger().warn('Waiting for /odometry/filtered ...')
            return
        if self._active_id is None or self._start is None:
            return
        s, st = self._filt, self._start
        dx, dy = s.x - st.x, s.y - st.y
        dyaw = deg(wrap_pi(s.yaw - st.yaw))
        self.get_logger().info(
            f'[T{self._active_id}] t={s.t - st.t:6.1f}s  '
            f'pose=({s.x:+.3f},{s.y:+.3f},{deg(s.yaw):+.1f}°)  '
            f'Δ=({dx:+.3f},{dy:+.3f},{dyaw:+.1f}°)  '
            f'|Δxy|={math.hypot(dx, dy):.3f}m'
        )

    def _on_help_srv(self, _req, res):
        self._print_help()
        res.success = True
        res.message = 'help printed'
        return res

    def _on_cmd(self, msg: String) -> None:
        line = (msg.data or '').strip()
        if not line:
            return
        parts = line.split()
        cmd, args = parts[0].lower(), parts[1:]
        self.get_logger().info(f'CMD: {line}')

        if cmd == 'help':
            self._print_help()
        elif cmd == 'status':
            self._print_status()
        elif cmd == 'start':
            if not args:
                self.get_logger().error('usage: start <1|2|3|drive>')
                return
            self._start_test(args[0].lower())
        elif cmd == 'mark':
            self._mark()
        elif cmd == 'done':
            self._finish_test()
        elif cmd == 'truth':
            if len(args) < 3:
                self.get_logger().error('usage: truth <x_m> <y_m> <yaw_deg>')
                return
            self._truth = (float(args[0]), float(args[1]), math.radians(float(args[2])))
            self.get_logger().info(
                f'Truth: x={self._truth[0]:.3f} y={self._truth[1]:.3f} '
                f'yaw={deg(self._truth[2]):.1f}°'
            )
        elif cmd == 'report':
            self._print_report()
        elif cmd == 'reset':
            self._results.clear()
            self._active_id = None
            self.get_logger().info('Results cleared')
        else:
            self.get_logger().error(f'Unknown cmd: {cmd} (try help)')

    def _print_help(self) -> None:
        self.get_logger().info('--- EKF CORE TESTS ---')
        self.get_logger().info(
            "  ros2 topic pub --once /ekf_test/cmd std_msgs/msg/String \"{data: 'start 1'}\""
        )
        for tid, desc in TEST_HELP.items():
            self.get_logger().info(f'  {tid}: {desc}')
        self.get_logger().info('  mark | done | status | truth x y yaw_deg | report | reset')

    def _print_status(self) -> None:
        if self._filt is None:
            self.get_logger().warn('No /odometry/filtered yet')
            return
        s = self._filt
        self.get_logger().info(
            f'FILTERED  x={s.x:+.4f} y={s.y:+.4f} yaw={deg(s.yaw):+.2f}°  '
            f'cov=({s.cov_x:.4g},{s.cov_y:.4g},{s.cov_yaw:.4g})'
        )
        if self._wheel:
            w = self._wheel
            self.get_logger().info(
                f'WHEEL     x={w.x:+.4f} y={w.y:+.4f} yaw={deg(w.yaw):+.2f}°'
            )

    def _start_test(self, tid: str) -> None:
        if tid not in TEST_HELP:
            self.get_logger().error('test must be 1, 2, 3, or drive')
            return
        if self._filt is None:
            self.get_logger().error('No /odometry/filtered — is EKF running?')
            return
        self._active_id = tid
        self._start = self._filt
        self._marks = [self._filt]
        self._samples = [self._filt]
        self._cov_trace = [(self._filt.t, self._filt.cov_x, self._filt.cov_y, self._filt.cov_yaw)]
        self._truth = None
        self.get_logger().info('*' * 50)
        self.get_logger().info(f'STARTED {tid}: {TEST_HELP[tid]}')
        self.get_logger().info(
            f'Start: x={self._start.x:+.4f} y={self._start.y:+.4f} '
            f'yaw={deg(self._start.yaw):+.2f}°'
        )
        if tid == '3':
            self.get_logger().info('Square: mark at each corner, done at start')
        self.get_logger().info('*' * 50)

    def _mark(self) -> None:
        if self._active_id is None or self._filt is None or self._start is None:
            self.get_logger().error('No active test')
            return
        self._marks.append(self._filt)
        s, st = self._filt, self._start
        self.get_logger().info(
            f'MARK #{len(self._marks) - 1}: x={s.x:+.4f} y={s.y:+.4f} '
            f'yaw={deg(s.yaw):+.2f}° |Δxy|={math.hypot(s.x - st.x, s.y - st.y):.4f}m'
        )

    def _finish_test(self) -> None:
        if self._active_id is None or self._start is None or self._filt is None:
            self.get_logger().error('No active test')
            return
        tid = self._active_id
        result = self._score(tid, self._start, self._filt)
        self._results.append(result)
        self._active_id = None

        tag = 'PASS' if result.passed else ('FAIL' if result.passed is False else 'INFO')
        line = f'Test {tid} {result.name}: {tag} — {result.summary}'
        self.get_logger().info('=' * 50)
        self.get_logger().info(line)
        for k, v in result.details.items():
            self.get_logger().info(f'  {k}: {v}')
        self.get_logger().info('=' * 50)
        with RESULTS_PATH.open('a') as f:
            f.write(line + '\n')
            for k, v in result.details.items():
                f.write(f'  {k}: {v}\n')

        if self._truth is not None:
            tx, ty, tyaw = self._truth
            end = self._filt
            self.get_logger().info(
                f'TRUTH error: |ex|={abs(end.x - tx):.3f}m '
                f'|ey|={abs(end.y - ty):.3f}m '
                f'|eyaw|={abs(deg(wrap_pi(end.yaw - tyaw))):.1f}°'
            )

    def _print_report(self) -> None:
        self.get_logger().info('=== EKF TEST REPORT ===')
        if not self._results:
            self.get_logger().info('(no completed tests)')
            return
        for r in self._results:
            tag = 'PASS' if r.passed else ('FAIL' if r.passed is False else 'INFO')
            self.get_logger().info(f'  T{r.test_id} {r.name}: {tag} — {r.summary}')

    def _score(self, tid: str, start: PoseSample, end: PoseSample) -> TestResult:
        dx, dy = end.x - start.x, end.y - start.y
        dxy = math.hypot(dx, dy)
        dyaw = wrap_pi(end.yaw - start.yaw)
        elapsed = end.t - start.t
        name = TEST_HELP[tid].split('—')[0].strip()

        if tid == '1':
            ok = abs(dx) < 0.03 and abs(dy) < 0.03 and abs(dyaw) < math.radians(5.0)
            return TestResult(
                tid, name, ok,
                f'{elapsed:.0f}s drift dx={dx * 100:+.1f}cm dy={dy * 100:+.1f}cm '
                f'dyaw={deg(dyaw):+.2f}° (limit ±3cm, ±5°)',
                {'dx_m': dx, 'dy_m': dy, 'dyaw_deg': deg(dyaw), 'elapsed_s': elapsed},
            )

        if tid == '2':
            if self._truth:
                tx, ty, _ = self._truth
                ex, ey = abs(end.x - tx), abs(end.y - ty)
            else:
                ex, ey = abs(dx - 2.0), abs(dy)
            ok = ex < 0.15 and ey < 0.10
            return TestResult(
                tid, name, ok,
                f'Δ=({dx:+.3f},{dy:+.3f}) err_vs_2m=({ex:.3f},{ey:.3f})m',
                {'dx_m': dx, 'dy_m': dy, 'err_x_m': ex, 'err_y_m': ey,
                 'dyaw_deg': deg(dyaw)},
            )

        if tid == '3':
            ok = dxy < 0.10 and abs(dyaw) < math.radians(15.0)
            return TestResult(
                tid, name, ok,
                f'closing |Δxy|={dxy * 100:.1f}cm dyaw={deg(dyaw):+.1f}° '
                f'marks={len(self._marks)} (good <10cm)',
                {'closing_m': dxy, 'dyaw_deg': deg(dyaw),
                 'marks': float(len(self._marks))},
            )

        # free drive
        jumps = 0
        for i in range(1, len(self._samples)):
            a, b = self._samples[i - 1], self._samples[i]
            if math.hypot(b.x - a.x, b.y - a.y) > 0.15:
                jumps += 1
        max_cov = 0.0
        if self._cov_trace:
            max_cov = max(max(c[1], c[2], c[3]) for c in self._cov_trace)
        ok = jumps == 0 and max_cov < 100.0
        return TestResult(
            tid, name, ok,
            f'{elapsed:.0f}s samples={len(self._samples)} jumps={jumps} '
            f'max_cov={max_cov:.4g} closing={dxy * 100:.1f}cm',
            {'jumps': float(jumps), 'max_cov': max_cov, 'closing_m': dxy,
             'dyaw_deg': deg(dyaw)},
        )


def main() -> None:
    rclpy.init()
    node = EkfMecanumTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
