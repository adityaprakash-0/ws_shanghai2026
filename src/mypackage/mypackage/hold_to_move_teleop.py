"""Dead-man keyboard teleop using Linux key press/release events."""

import errno
import glob
import os
import struct
import sys
import termios
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


EV_KEY = 0x01
KEY_ESC = 1
KEY_MINUS = 12
KEY_EQUAL = 13
KEY_Q = 16
KEY_W = 17
KEY_E = 18
KEY_A = 30
KEY_S = 31
KEY_D = 32
KEY_SPACE = 57

MOTION_KEYS = {KEY_Q, KEY_W, KEY_E, KEY_A, KEY_S, KEY_D}
INPUT_EVENT = struct.Struct('@llHHi')


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def find_keyboard_device(requested: str) -> str:
    if requested:
        return os.path.realpath(os.path.expanduser(requested))

    keyboards = sorted(glob.glob('/dev/input/by-id/*-event-kbd'))
    if keyboards:
        return os.path.realpath(keyboards[0])

    raise RuntimeError(
        'No keyboard found under /dev/input/by-id. Pass the device explicitly, '
        'for example: -p device:=/dev/input/event3')


class HoldToMoveTeleop(Node):
    """Publish velocity only while movement keys are physically held."""

    def __init__(self) -> None:
        super().__init__('hold_to_move_teleop')

        self.declare_parameter('device', '')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('min_linear_speed', 0.10)
        self.declare_parameter('max_linear_speed', 0.30)
        self.declare_parameter('initial_linear_speed', 0.15)
        self.declare_parameter('linear_speed_step', 0.05)
        self.declare_parameter('min_angular_speed', 0.10)
        self.declare_parameter('max_angular_speed', 0.30)
        self.declare_parameter('initial_angular_speed', 0.15)
        self.declare_parameter('angular_speed_step', 0.05)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('deadman_timeout', 0.0)

        self.min_linear = float(self.get_parameter('min_linear_speed').value)
        self.max_linear = float(self.get_parameter('max_linear_speed').value)
        self.min_angular = float(self.get_parameter('min_angular_speed').value)
        self.max_angular = float(self.get_parameter('max_angular_speed').value)
        if self.min_linear < 0.0 or self.max_linear < self.min_linear:
            raise ValueError('Require 0 <= min_linear_speed <= max_linear_speed')
        if self.min_angular < 0.0 or self.max_angular < self.min_angular:
            raise ValueError('Require 0 <= min_angular_speed <= max_angular_speed')

        self.linear_step = max(
            0.0, float(self.get_parameter('linear_speed_step').value))
        self.angular_step = max(
            0.0, float(self.get_parameter('angular_speed_step').value))
        self.linear_speed = clamp(
            float(self.get_parameter('initial_linear_speed').value),
            self.min_linear, self.max_linear)
        self.angular_speed = clamp(
            float(self.get_parameter('initial_angular_speed').value),
            self.min_angular, self.max_angular)

        topic = str(self.get_parameter('cmd_vel_topic').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        if publish_rate <= 0.0:
            raise ValueError('publish_rate must be greater than zero')
        self.deadman_timeout = max(
            0.0, float(self.get_parameter('deadman_timeout').value))

        requested_device = str(self.get_parameter('device').value)
        self.device = find_keyboard_device(requested_device)
        self.fd = os.open(self.device, os.O_RDONLY | os.O_NONBLOCK)
        self.publisher = self.create_publisher(Twist, topic, 10)
        self.held_keys = set()
        self.last_event_time = self.get_clock().now()
        self.running = True

        self.create_timer(0.005, self._read_events)
        self.create_timer(1.0 / publish_rate, self._publish_command)

        self.get_logger().info(
            f'Hold-to-move teleop ready on {self.device} -> {topic}\n'
            '  W/S: forward/reverse | A/D: left/right strafe\n'
            '  Q/E: rotate left/right | release keys: immediate stop\n'
            '  +/-: adjust speeds | SPACE: stop | ESC: exit')
        self._log_speeds()

    def _log_speeds(self) -> None:
        self.get_logger().info(
            f'Linear speed {self.linear_speed:.2f} '
            f'[{self.min_linear:.2f}, {self.max_linear:.2f}], '
            f'angular speed {self.angular_speed:.2f} '
            f'[{self.min_angular:.2f}, {self.max_angular:.2f}]')

    def _read_events(self) -> None:
        while self.running:
            try:
                data = os.read(self.fd, INPUT_EVENT.size * 32)
            except BlockingIOError:
                return
            except OSError as exc:
                if exc.errno in (errno.ENODEV, errno.EBADF):
                    self.get_logger().error('Keyboard disconnected; stopping robot')
                    self.held_keys.clear()
                    self.publish_stop()
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
        # value: 0=release, 1=press, 2=hardware auto-repeat
        self.last_event_time = self.get_clock().now()

        if code in MOTION_KEYS:
            if value == 0:
                self.held_keys.discard(code)
                if not self.held_keys:
                    self.publish_stop()
            else:
                self.held_keys.add(code)
            return

        if value != 1:
            return

        if code == KEY_SPACE:
            self.held_keys.clear()
            self.publish_stop()
        elif code == KEY_ESC:
            self.held_keys.clear()
            self.publish_stop()
            self.running = False
            rclpy.shutdown()
        elif code == KEY_EQUAL:
            self.linear_speed = clamp(
                self.linear_speed + self.linear_step,
                self.min_linear, self.max_linear)
            self.angular_speed = clamp(
                self.angular_speed + self.angular_step,
                self.min_angular, self.max_angular)
            self._log_speeds()
        elif code == KEY_MINUS:
            self.linear_speed = clamp(
                self.linear_speed - self.linear_step,
                self.min_linear, self.max_linear)
            self.angular_speed = clamp(
                self.angular_speed - self.angular_step,
                self.min_angular, self.max_angular)
            self._log_speeds()

    def _publish_command(self) -> None:
        if not self.running:
            return

        age = (self.get_clock().now() - self.last_event_time).nanoseconds / 1e9
        if (
                self.deadman_timeout > 0.0
                and self.held_keys
                and age > self.deadman_timeout):
            self.get_logger().warn('Keyboard event timeout; stopping robot')
            self.held_keys.clear()

        twist = Twist()
        twist.linear.x = self.linear_speed * (
            int(KEY_W in self.held_keys) - int(KEY_S in self.held_keys))
        twist.linear.y = self.linear_speed * (
            int(KEY_A in self.held_keys) - int(KEY_D in self.held_keys))
        twist.angular.z = self.angular_speed * (
            int(KEY_Q in self.held_keys) - int(KEY_E in self.held_keys))
        self.publisher.publish(twist)

    def publish_stop(self) -> None:
        stop = Twist()
        # Publish more than once so a best-effort subscriber cannot miss the stop.
        for _ in range(3):
            self.publisher.publish(stop)

    def close(self) -> None:
        self.held_keys.clear()
        if rclpy.ok():
            self.publish_stop()
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
        node = HoldToMoveTeleop()
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
            termios.tcsetattr(
                sys.stdin, termios.TCSADRAIN, terminal_settings)


if __name__ == '__main__':
    main()
