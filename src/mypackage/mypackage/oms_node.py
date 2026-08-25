"""OMS click-to-run teleop.

One key press = one service call. Nothing is held. Continuous axes keep
running until SPACE (or the stop key) sends zero. Standard servos only
have two poses each.

  Titans (service /titan1/titan_cmd, params=set_speed):
    Q / E  rotate ACW / CW     motor 0
    R / F  lift up / down      motor 2
    SPACE  stop rotate + lift + linear (speed 0)

  Servos (service /<name>/set_servo, initparams.speed):
    T / G  linear fwd / back   continuous; SPACE → 0
    Y / H  vertical up / down  two angles
    U / J  rotation horiz/vert two angles
    I / K  gripper open/close  two angles

  ros2 launch mypackage oms.launch.py
  ros2 launch mypackage oms.launch.py enable_servos:=true
"""

from __future__ import annotations

import errno
import glob
import os
import struct
import sys
import termios
from typing import Optional

import rclpy
from rclpy.node import Node
from studica_control.srv import SetData


EV_KEY = 0x01
KEY_ESC = 1
KEY_Q = 16
KEY_E = 18
KEY_R = 19
KEY_T = 20
KEY_Y = 21
KEY_U = 22
KEY_I = 23
KEY_F = 33
KEY_G = 34
KEY_H = 35
KEY_J = 36
KEY_K = 37
KEY_SPACE = 57

INPUT_EVENT = struct.Struct('@llHHi')

ROTATE_MOTOR = 0
LIFT_MOTOR = 2


def find_keyboard_device(requested: str) -> str:
    if requested:
        return os.path.realpath(os.path.expanduser(requested))

    keyboards = sorted(glob.glob('/dev/input/by-id/*-event-kbd'))
    if keyboards:
        return os.path.realpath(keyboards[0])

    raise RuntimeError(
        'No keyboard found under /dev/input/by-id. Pass the device explicitly, '
        'for example: -p device:=/dev/input/event3')


class OmsNode(Node):
    def __init__(self) -> None:
        super().__init__('oms_node')

        self.declare_parameter('device', '')
        self.declare_parameter('titan_service', '/titan1/titan_cmd')
        self.declare_parameter('linear_service', '/linear_servo/set_servo')
        self.declare_parameter('vertical_service', '/vertical_servo/set_servo')
        self.declare_parameter('rotation_service', '/rotation_servo/set_servo')
        self.declare_parameter('gripper_service', '/gripper_servo/set_servo')
        self.declare_parameter('rotate_duty', 0.30)
        self.declare_parameter('lift_duty', 0.30)
        self.declare_parameter('linear_speed', 50.0)
        self.declare_parameter('vertical_up_deg', 90.0)
        self.declare_parameter('vertical_down_deg', 0.0)
        self.declare_parameter('rotation_horizontal_deg', 0.0)
        self.declare_parameter('rotation_vertical_deg', 90.0)
        self.declare_parameter('gripper_open_deg', 45.0)
        self.declare_parameter('gripper_close_deg', -45.0)

        self.rotate_duty = float(self.get_parameter('rotate_duty').value)
        self.lift_duty = float(self.get_parameter('lift_duty').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.vertical_up = float(self.get_parameter('vertical_up_deg').value)
        self.vertical_down = float(self.get_parameter('vertical_down_deg').value)
        self.rotation_horizontal = float(
            self.get_parameter('rotation_horizontal_deg').value)
        self.rotation_vertical = float(
            self.get_parameter('rotation_vertical_deg').value)
        self.gripper_open = float(self.get_parameter('gripper_open_deg').value)
        self.gripper_close = float(self.get_parameter('gripper_close_deg').value)

        self.titan_cli = self.create_client(
            SetData, str(self.get_parameter('titan_service').value))
        self.linear_cli = self.create_client(
            SetData, str(self.get_parameter('linear_service').value))
        self.vertical_cli = self.create_client(
            SetData, str(self.get_parameter('vertical_service').value))
        self.rotation_cli = self.create_client(
            SetData, str(self.get_parameter('rotation_service').value))
        self.gripper_cli = self.create_client(
            SetData, str(self.get_parameter('gripper_service').value))

        self.running = True
        requested_device = str(self.get_parameter('device').value)
        self.device = find_keyboard_device(requested_device)
        self.fd = os.open(self.device, os.O_RDONLY | os.O_NONBLOCK)

        self.create_timer(0.005, self._read_events)

        self.get_logger().info(
            f'OMS click-to-run on {self.device}\n'
            '  Q/E  rotate ACW / CW   (titan1 m_0, stays until SPACE)\n'
            '  R/F  lift up / down    (titan1 m_2, stays until SPACE)\n'
            '  T/G  linear fwd / back (continuous servo, stays until SPACE)\n'
            '  Y/H  vertical up / down\n'
            '  U/J  rotation horiz / vert\n'
            '  I/K  gripper open / close\n'
            '  SPACE stop continuous (motors + linear = 0) | ESC quit')

    def _read_events(self) -> None:
        while self.running:
            try:
                data = os.read(self.fd, INPUT_EVENT.size * 32)
            except BlockingIOError:
                return
            except OSError as exc:
                if exc.errno in (errno.ENODEV, errno.EBADF):
                    self.get_logger().error('Keyboard disconnected; stopping OMS')
                    self._stop_continuous()
                    self.running = False
                    rclpy.shutdown()
                    return
                raise

            if not data:
                return

            complete_length = len(data) - (len(data) % INPUT_EVENT.size)
            for offset in range(0, complete_length, INPUT_EVENT.size):
                _, _, event_type, code, value = INPUT_EVENT.unpack_from(data, offset)
                if event_type == EV_KEY:
                    self._handle_key(code, value)

    def _handle_key(self, code: int, value: int) -> None:
        # 0=release, 1=press, 2=auto-repeat — click means press only
        if value != 1:
            return

        if code == KEY_Q:
            self._titan(ROTATE_MOTOR, -self.rotate_duty, 'rotate ACW')
        elif code == KEY_E:
            self._titan(ROTATE_MOTOR, self.rotate_duty, 'rotate CW')
        elif code == KEY_R:
            self._titan(LIFT_MOTOR, self.lift_duty, 'lift up')
        elif code == KEY_F:
            self._titan(LIFT_MOTOR, -self.lift_duty, 'lift down')
        elif code == KEY_T:
            self._servo(self.linear_cli, self.linear_speed, 'linear forward')
        elif code == KEY_G:
            self._servo(self.linear_cli, -self.linear_speed, 'linear backward')
        elif code == KEY_Y:
            self._servo(self.vertical_cli, self.vertical_up, 'vertical up')
        elif code == KEY_H:
            self._servo(self.vertical_cli, self.vertical_down, 'vertical down')
        elif code == KEY_U:
            self._servo(self.rotation_cli, self.rotation_horizontal, 'rotation horizontal')
        elif code == KEY_J:
            self._servo(self.rotation_cli, self.rotation_vertical, 'rotation vertical')
        elif code == KEY_I:
            self._servo(self.gripper_cli, self.gripper_open, 'gripper open')
        elif code == KEY_K:
            self._servo(self.gripper_cli, self.gripper_close, 'gripper close')
        elif code == KEY_SPACE:
            self._stop_continuous()
        elif code == KEY_ESC:
            self._stop_continuous()
            self.running = False
            rclpy.shutdown()

    def _titan(self, motor: int, duty: float, label: str) -> None:
        if not self.titan_cli.service_is_ready():
            self.get_logger().warn(f'{label}: /titan1/titan_cmd not ready')
            return
        req = SetData.Request()
        req.params = 'set_speed'
        req.initparams.n_encoder = motor
        req.initparams.speed = float(duty)
        future = self.titan_cli.call_async(req)
        future.add_done_callback(
            lambda f, lab=label, d=duty: self._on_done(lab, d, f))

    def _servo(self, client, value: float, label: str) -> None:
        if not client.service_is_ready():
            self.get_logger().warn(
                f'{label}: servo service not ready (enable_servos:=true?)')
            return
        req = SetData.Request()
        req.initparams.speed = float(value)
        future = client.call_async(req)
        future.add_done_callback(
            lambda f, lab=label, v=value: self._on_done(lab, v, f))

    def _on_done(self, label: str, value: float, future) -> None:
        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().error(f'{label}: {exc}')
            return
        if resp.success:
            self.get_logger().info(f'{label} → {value:g}  ({resp.message})')
        else:
            self.get_logger().error(f'{label} failed: {resp.message}')

    def _stop_continuous(self) -> None:
        self._titan(ROTATE_MOTOR, 0.0, 'rotate stop')
        self._titan(LIFT_MOTOR, 0.0, 'lift stop')
        self._servo(self.linear_cli, 0.0, 'linear stop')
        self.get_logger().info('SPACE: continuous axes set to 0')

    def close(self) -> None:
        if rclpy.ok():
            self._stop_continuous()
        if getattr(self, 'fd', -1) >= 0:
            os.close(self.fd)
            self.fd = -1


def disable_terminal_echo() -> Optional[list]:
    if not sys.stdin.isatty():
        return None
    original = termios.tcgetattr(sys.stdin)
    updated = termios.tcgetattr(sys.stdin)
    updated[3] &= ~(termios.ECHO | termios.ICANON)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, updated)
    return original


def main(args=None) -> None:
    terminal_settings = disable_terminal_echo()
    rclpy.init(args=args)
    node = None
    try:
        node = OmsNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if terminal_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal_settings)


if __name__ == '__main__':
    main()
