#!/usr/bin/env python3
"""OMS bench CLI — rotation, lift, linear, vertical, rotation servo, gripper.

Home sequence (allhome / dashboard Home): lift → linear → vertical (down)
→ gripper → rotation_servo (in −30°).

Terminal 1:  ros2 launch oms test_oms.launch.py
Terminal 2:  ros2 run oms oms_test

Type:
  30              rotate OMS to +30° (ACW) from home
  -45             rotate to −45° (CW)
  0               rotate back to home angle
  home            search rotate limit switch and mark 0°
  lift 50         chain lift to 50 mm down from up limit (0 = home/up)
  lift 0          lift to home (up limit)
  lift home       search up limit and reset encoder
  ldown           drive to lower limit (floor)
  lstop           stop lift motor
  linear 70       linear axis to 70 mm forward from home (max 100 mm)
  linear 30       linear axis to 30 mm forward from home
  linear home     backward to M0 limit_fwd, reset M3 encoder once
  linear stop     stop linear servo
  up / down       vertical servo (90° / −90°)
  in / out        rotation servo in (−30°) / out (−150°)
  open / close    gripper open (−80°) / closed (150°)
  allhome         full boot home sequence
  stop            zero rotate duty
  q               quit

One-shot:
  ros2 run oms oms_test -- linear 70
  ros2 run oms oms_test -- linear home
  ros2 run oms oms_test -- lift 100
  ros2 run oms oms_test -- allhome
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
        self._lift_pos = None
        self._lift_state = ''
        self._linear_pos = None
        self._linear_state = ''
        self._rot_servo_pose = ''
        self._gripper_pose = ''
        self._last_result = None
        self._cmd_pub = self.create_publisher(String, '/oms/rotation/cmd', 10)
        self._lift_pub = self.create_publisher(String, '/oms/lift/cmd', 10)
        self._linear_pub = self.create_publisher(String, '/oms/linear/cmd', 10)
        self._vert_pub = self.create_publisher(String, '/oms/vertical/cmd', 10)
        self._rot_servo_pub = self.create_publisher(String, '/oms/rotation_servo/cmd', 10)
        self._gripper_pub = self.create_publisher(String, '/oms/gripper/cmd', 10)
        self._home_pub = self.create_publisher(String, '/oms/home/cmd', 10)
        self.create_subscription(
            String, '/oms/rotation/status', self._on_status, 10)
        self.create_subscription(
            String, '/oms/rotation/result', self._on_result, 10)
        self.create_subscription(
            String, '/oms/lift/status', self._on_lift_status, 10)
        self.create_subscription(
            String, '/oms/lift/result', self._on_result, 10)
        self.create_subscription(
            String, '/oms/linear/status', self._on_linear_status, 10)
        self.create_subscription(
            String, '/oms/linear/result', self._on_result, 10)
        self.create_subscription(
            String, '/oms/vertical/status', self._on_vert_status, 10)
        self.create_subscription(
            String, '/oms/vertical/result', self._on_result, 10)
        self.create_subscription(
            String, '/oms/rotation_servo/status', self._on_rot_servo_status, 10)
        self.create_subscription(
            String, '/oms/rotation_servo/result', self._on_result, 10)
        self.create_subscription(
            String, '/oms/gripper/status', self._on_gripper_status, 10)
        self.create_subscription(
            String, '/oms/gripper/result', self._on_result, 10)
        self.create_subscription(
            String, '/oms/home/result', self._on_result, 10)
        self.create_subscription(
            Float64, '/oms/rotation/angle_deg', self._on_angle, 10)
        self.create_subscription(
            Float64, '/oms/lift/pos_mm', self._on_lift_pos, 10)
        self.create_subscription(
            Float64, '/oms/linear/pos_mm', self._on_linear_pos, 10)
        self.get_logger().info(
            'OMS test — angle | home | lift | linear | up/down | '
            'in/out | open/close | allhome | stop | q')

    def _on_angle(self, msg: Float64) -> None:
        self._angle = float(msg.data)

    def _on_lift_pos(self, msg: Float64) -> None:
        self._lift_pos = float(msg.data)

    def _on_linear_pos(self, msg: Float64) -> None:
        self._linear_pos = float(msg.data)

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

    def _on_lift_status(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if not isinstance(data, dict):
            return
        self._lift_state = str(data.get('state') or '')
        if data.get('pos_mm') is not None:
            try:
                self._lift_pos = float(data['pos_mm'])
            except (TypeError, ValueError):
                pass

    def _on_linear_status(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if not isinstance(data, dict):
            return
        self._linear_state = str(data.get('state') or '')
        if data.get('pos_mm') is not None:
            try:
                self._linear_pos = float(data['pos_mm'])
            except (TypeError, ValueError):
                pass

    def _on_vert_status(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if isinstance(data, dict):
            self._vert_pose = str(data.get('pose') or '')

    def _on_rot_servo_status(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if isinstance(data, dict):
            self._rot_servo_pose = str(data.get('pose') or '')

    def _on_gripper_status(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if isinstance(data, dict):
            self._gripper_pose = str(data.get('pose') or '')

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
        pos = data.get('pos_mm')
        pose = data.get('pose')
        extra = ''
        if ang is not None:
            extra += f'  angle={float(ang):+.2f}°'
        if pos is not None:
            mode = str(data.get('mode') or '')
            if mode == 'linear':
                extra += f'  linear={float(pos):.1f}mm'
            else:
                extra += f'  lift={float(pos):.1f}mm'
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
        if low in ('lstop', 'lift stop'):
            self._send(self._lift_pub, {'mode': 'stop'})
            return True
        if low in ('linear stop', 'linstop'):
            self._send(self._linear_pub, {'mode': 'stop'})
            return True
        if low == 'home':
            self._send(self._cmd_pub, {'mode': 'home'})
            return True
        if low in ('allhome', 'home all', 'boot home'):
            self._send(self._home_pub, {'mode': 'home'})
            return True
        if low in ('lift home', 'lhome'):
            self._send(self._lift_pub, {'mode': 'home'})
            return True
        if low in ('ldown', 'lift down'):
            self._send(self._lift_pub, {'mode': 'down'})
            return True
        if low in ('linear home', 'lhome linear'):
            self._send(self._linear_pub, {'mode': 'home'})
            return True
        if low in ('up', 'down'):
            self._send(self._vert_pub, {'mode': low})
            return True
        if low in ('in', 'i', 'horiz', 'horizontal', 'h'):
            self._send(self._rot_servo_pub, {'mode': 'in'})
            return True
        if low in ('out', 'o', 'vert', 'vertical', 'v'):
            self._send(self._rot_servo_pub, {'mode': 'out'})
            return True
        if low in ('open', 'grip open', 'gripper open'):
            self._send(self._gripper_pub, {'mode': 'open'})
            return True
        if low in ('close', 'grip close', 'gripper close'):
            self._send(self._gripper_pub, {'mode': 'close'})
            return True
        if low.startswith('lift '):
            arg = line[5:].strip()
            if arg.lower() in ('home', 'up'):
                self._send(self._lift_pub, {'mode': 'home'})
                return True
            if arg.lower() == 'down':
                self._send(self._lift_pub, {'mode': 'down'})
                return True
            try:
                pos = float(arg)
            except ValueError:
                self.get_logger().warn(f'bad lift target {arg!r}')
                return True
            self._send(self._lift_pub, {'mode': 'goto', 'pos_mm': pos})
            return True
        if low.startswith('linear '):
            arg = line[7:].strip()
            if arg.lower() in ('home', 'back'):
                self._send(self._linear_pub, {'mode': 'home'})
                return True
            if arg.lower() == 'stop':
                self._send(self._linear_pub, {'mode': 'stop'})
                return True
            try:
                pos = float(arg)
            except ValueError:
                self.get_logger().warn(f'bad linear target {arg!r}')
                return True
            self._send(self._linear_pub, {'mode': 'goto', 'pos_mm': pos})
            return True
        try:
            angle = float(line)
        except ValueError:
            self.get_logger().warn(
                f'unknown command {line!r} — angle, home, lift, linear, '
                'up/down, in/out, open/close, allhome, stop, q')
            return True
        self._send(self._cmd_pub, {'mode': 'goto', 'angle_deg': angle})
        return True

    def _print_status(self) -> None:
        ang = '—' if self._angle is None else f'{self._angle:+.2f}°'
        lift = '—' if self._lift_pos is None else f'{self._lift_pos:.1f}mm'
        linear = '—' if self._linear_pos is None else f'{self._linear_pos:.1f}mm'
        self.get_logger().info(
            f'rotate state={self._state or "?"} homed={self._homed} angle={ang}  '
            f'lift state={self._lift_state or "?"} pos={lift}  '
            f'linear state={self._linear_state or "?"} pos={linear}  '
            f'vertical={self._vert_pose or "?"}  '
            f'rot_servo={self._rot_servo_pose or "?"}  '
            f'gripper={self._gripper_pose or "?"}')

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
        'OMS test  angle | home | lift | linear | up/down | '
        'in/out | open/close | allhome | stop | q',
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
            timeout = 120.0 if oneshot.lower().startswith('allhome') else 60.0
            if oneshot.lower().startswith('linear'):
                timeout = 90.0
            node.wait_result(timeout)
        else:
            _prompt_loop(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
