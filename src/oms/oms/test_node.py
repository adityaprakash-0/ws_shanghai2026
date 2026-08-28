#!/usr/bin/env python3
"""OMS bench CLI — rotation + vertical from a second terminal.

Terminal 1:  ros2 launch oms test_oms.launch.py
Terminal 2:  ros2 run oms oms_test

Type:
  30        rotate OMS to +30° (ACW) from home
  -45       rotate to −45° (CW)
  0         back to home angle via PID
  home      search the rotate limit switch and mark 0°
  up        vertical servo up (50°, same as teleop Y)
  down      vertical servo down / home (−40°, same as teleop H)
  stop      zero rotate duty
  q         quit

One-shot:
  ros2 run oms oms_test -- 30
  ros2 run oms oms_test -- up
  ros2 run oms oms_test -- down
"""

from __future__ import annotations

import json
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String


def _argv_command() -> str:
    extra = []
    skip = False
    for arg in sys.argv[1:]:
        if skip:
            skip = False
            continue
        if arg in ('--ros-args', '--params-file'):
            continue
        if arg == '--':
            continue
        if arg.startswith('-p') or arg.startswith('--'):
            skip = arg in ('-p', '--params-file')
            continue
        extra.append(arg)
    return ' '.join(extra).strip()


class OmsTestNode(Node):
    def __init__(self) -> None:
        super().__init__('oms_test')
        self._angle = None
        self._homed = False
        self._state = ''
        self._vert_pose = ''
        self._last_result = None
        self._cmd_pub = self.create_publisher(String, '/oms/rotation/cmd', 10)
        self._vert_pub = self.create_publisher(String, '/oms/vertical/cmd', 10)
        self.create_subscription(
            String, '/oms/rotation/status', self._on_status, 10)
        self.create_subscription(
            String, '/oms/rotation/result', self._on_result, 10)
        self.create_subscription(
            String, '/oms/vertical/status', self._on_vert_status, 10)
        self.create_subscription(
            String, '/oms/vertical/result', self._on_result, 10)
        self.create_subscription(
            Float64, '/oms/rotation/angle_deg', self._on_angle, 10)
        self.get_logger().info(
            'OMS test — angle | home | up | down | stop | q')

    def _on_angle(self, msg: Float64) -> None:
        self._angle = float(msg.data)

    def _on_status(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if not isinstance(data, dict):
            return
        self._state = str(data.get('state') or '')
        self._homed = bool(data.get('homed'))
        if data.get('angle_deg') is not None:
            try:
                self._angle = float(data['angle_deg'])
            except (TypeError, ValueError):
                pass

    def _on_vert_status(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if isinstance(data, dict):
            self._vert_pose = str(data.get('pose') or '')

    def _on_result(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if not isinstance(data, dict):
            return
        self._last_result = data
        ok = bool(data.get('success'))
        ang = data.get('angle_deg')
        pose = data.get('pose')
        extra = ''
        if ang is not None:
            extra += f'  angle={float(ang):+.2f}°'
        if pose:
            extra += f'  pose={pose}'
        self.get_logger().info(
            f'RESULT ok={ok} {data.get("message") or ""}{extra}')

    def _send(self, pub, payload: dict) -> None:
        payload.setdefault('cmd_id', f'test_{int(time.time() * 1000)}')
        pub.publish(String(data=json.dumps(payload)))
        self.get_logger().info(f'SEND {payload}')

    def handle_line(self, raw: str) -> bool:
        """Return False to quit."""
        line = (raw or '').strip()
        if not line:
            self._print_status()
            return True
        low = line.lower()
        if low in ('q', 'quit', 'exit'):
            return False
        if low in ('stop', 'cancel'):
            self._send(self._cmd_pub, {'mode': 'stop'})
            return True
        if low == 'home':
            self._send(self._cmd_pub, {'mode': 'home'})
            return True
        if low in ('up', 'down'):
            self._send(self._vert_pub, {'mode': low})
            return True
        try:
            angle = float(line)
        except ValueError:
            self.get_logger().warn(
                f'unknown command {line!r} — angle, home, up, down, stop, q')
            return True
        self._send(self._cmd_pub, {'mode': 'goto', 'angle_deg': angle})
        return True

    def _print_status(self) -> None:
        ang = '—' if self._angle is None else f'{self._angle:+.2f}°'
        self.get_logger().info(
            f'rotate state={self._state or "?"} homed={self._homed} '
            f'angle={ang}  vertical={self._vert_pose or "?"}')

    def wait_result(self, timeout: float) -> bool:
        t0 = time.time()
        seen = self._last_result
        while rclpy.ok() and (time.time() - t0) < timeout:
            if self._last_result is not None and self._last_result is not seen:
                return bool(self._last_result.get('success'))
            time.sleep(0.05)
        self.get_logger().warn(f'no result after {timeout:.0f}s')
        return False


def _prompt_loop(node: OmsTestNode) -> None:
    time.sleep(0.4)
    print(
        'OMS test  angle | home | up | down | stop | q     '
        '(empty line = status)',
        flush=True)
    while rclpy.ok():
        try:
            line = input('oms> ')
        except EOFError:
            break
        except KeyboardInterrupt:
            break
        if not node.handle_line(line):
            break
    if rclpy.ok():
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OmsTestNode()
    oneshot = _argv_command()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    try:
        if oneshot:
            time.sleep(0.3)
            if not node.handle_line(oneshot):
                return
            node.wait_result(30.0)
        else:
            _prompt_loop(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
