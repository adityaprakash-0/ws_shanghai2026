"""OMS click-to-run teleop.

One key press = one service call. Nothing is held. Continuous axes keep
running until SPACE (or the stop key) sends zero. Standard servos only
have two poses each.

  Titans (service /titan1/titan_cmd, params=set_speed):
    Q / E  rotate ACW / CW     titan1 m_2
    R / F  lift up / down      titan1 m_1
    SPACE  stop rotate + lift + linear (speed 0)

  Servos:
    T / G  linear fwd / back   topic /linear_servo/cmd (continuous); SPACE → 0
    Y / H  vertical up / down  service (two angles)
    U / J  rotation in / out   service (two angles)
    I / K  gripper open/close  service (two angles)

  ros2 launch mypackage oms.launch.py enable_servos:=true
  # then in a second terminal (this is where you type):
  ros2 run mypackage oms_node
"""

from __future__ import annotations

import errno
import fcntl
import glob
import os
import struct
import sys
import termios
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
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

ROTATE_MOTOR = 2
LIFT_MOTOR = 1

STDIN_ALIASES = {'', 'auto', 'stdin', 'tty', '/dev/tty'}

CHAR_TO_CODE = {
    'q': KEY_Q, 'Q': KEY_Q,
    'e': KEY_E, 'E': KEY_E,
    'r': KEY_R, 'R': KEY_R,
    'f': KEY_F, 'F': KEY_F,
    't': KEY_T, 'T': KEY_T,
    'g': KEY_G, 'G': KEY_G,
    'y': KEY_Y, 'Y': KEY_Y,
    'h': KEY_H, 'H': KEY_H,
    'u': KEY_U, 'U': KEY_U,
    'j': KEY_J, 'J': KEY_J,
    'i': KEY_I, 'I': KEY_I,
    'k': KEY_K, 'K': KEY_K,
    ' ': KEY_SPACE,
    '\x1b': KEY_ESC,
}


def find_evdev_keyboard(requested: str) -> Optional[str]:
    raw = str(requested or '').strip()
    if raw.lower() in STDIN_ALIASES:
        for path in sorted(glob.glob('/dev/input/by-id/*-event-kbd')):
            real = os.path.realpath(path)
            if 'hdmi' in path.lower() or 'vc4' in real.lower():
                continue
            return real
        return None
    return os.path.realpath(os.path.expanduser(raw))


class OmsNode(Node):
    def __init__(self) -> None:
        super().__init__('oms_node')

        self.declare_parameter('device', '')
        self.declare_parameter('titan_service', '/titan1/titan_cmd')
        self.declare_parameter('linear_cmd_topic', '/linear_servo/cmd')
        self.declare_parameter('linear_speed', 30.0)
        self.declare_parameter('vertical_service', '/vertical_servo/set_servo')
        self.declare_parameter('rotation_service', '/rotation_servo/set_servo')
        self.declare_parameter('gripper_service', '/gripper_servo/set_servo')
        self.declare_parameter('rotate_duty', 0.2)
        self.declare_parameter('lift_duty', 0.50)
        self.declare_parameter('rotate_motor', ROTATE_MOTOR)
        self.declare_parameter('lift_motor', LIFT_MOTOR)
        self.declare_parameter('vertical_up_deg', 60.0)
        self.declare_parameter('vertical_down_deg', -40.0)
        self.declare_parameter('rotation_in_deg', 110.0)
        self.declare_parameter('rotation_out_deg', 0.0)
        self.declare_parameter('rotation_horizontal_deg', 110.0)
        self.declare_parameter('rotation_vertical_deg', 0.0)
        self.declare_parameter('gripper_open_deg', -150.0)
        self.declare_parameter('gripper_close_deg', 150.0)

        self.rotate_duty = float(self.get_parameter('rotate_duty').value)
        self.lift_duty = float(self.get_parameter('lift_duty').value)
        self.rotate_motor = int(self.get_parameter('rotate_motor').value)
        self.lift_motor = int(self.get_parameter('lift_motor').value)
        self.linear_speed = max(1.0, min(100.0, abs(float(
            self.get_parameter('linear_speed').value))))
        self.vertical_up = float(self.get_parameter('vertical_up_deg').value)
        self.vertical_down = float(self.get_parameter('vertical_down_deg').value)
        self.rotation_in = float(self.get_parameter('rotation_in_deg').value)
        self.rotation_out = float(self.get_parameter('rotation_out_deg').value)
        if abs(self.rotation_in - 110.0) < 1e-9:
            self.rotation_in = float(
                self.get_parameter('rotation_horizontal_deg').value)
        if abs(self.rotation_out - 0.0) < 1e-9:
            self.rotation_out = float(
                self.get_parameter('rotation_vertical_deg').value)
        self.gripper_open = float(self.get_parameter('gripper_open_deg').value)
        self.gripper_close = float(self.get_parameter('gripper_close_deg').value)

        self.titan_cli = self.create_client(
            SetData, str(self.get_parameter('titan_service').value))
        self.vertical_cli = self.create_client(
            SetData, str(self.get_parameter('vertical_service').value))
        self.rotation_cli = self.create_client(
            SetData, str(self.get_parameter('rotation_service').value))
        self.gripper_cli = self.create_client(
            SetData, str(self.get_parameter('gripper_service').value))

        linear_topic = str(self.get_parameter('linear_cmd_topic').value)
        self._linear_pub = self.create_publisher(Float64, linear_topic, 10)
        self._linear_cmd = 0

        self.running = True
        self.fd = -1
        self._own_fd = False
        self._tty_settings = None
        self._use_tty = False
        requested_device = str(self.get_parameter('device').value)
        evdev = find_evdev_keyboard(requested_device)
        if evdev:
            self.device = evdev
            self.fd = os.open(self.device, os.O_RDONLY | os.O_NONBLOCK)
            self._own_fd = True
            self.create_timer(0.005, self._read_events)
            source = self.device
        else:
            self._open_tty()
            self.create_timer(0.02, self._read_tty)
            source = f'terminal ({self.device})'

        self.get_logger().info(
            f'OMS click-to-run on {source}\n'
            f'  Q/E  rotate ACW / CW   (titan1 m_{self.rotate_motor}, stays until SPACE)\n'
            f'  R/F  lift up / down    (titan1 m_{self.lift_motor}, stays until SPACE)\n'
            '  T/G  linear fwd / back (continuous −100…100, latches until SPACE)\n'
            '  Y/H  vertical up / down\n'
            '  U/J  rotation in / out\n'
            '  I/K  gripper open / close\n'
            '  SPACE stop motors (node stays running) | ESC quit')

    def _open_tty(self) -> None:
        """Read keys from the SSH/terminal session (no USB keyboard on the Pi)."""
        if sys.stdin.isatty():
            self.device = 'stdin'
            self.fd = sys.stdin.fileno()
            self._own_fd = False
            self._use_tty = True
            self._make_raw(self.fd)
            return
        for candidate in ('/dev/tty',):
            try:
                fd = os.open(candidate, os.O_RDONLY | os.O_NONBLOCK)
            except OSError:
                continue
            self.device = candidate
            self.fd = fd
            self._own_fd = True
            self._use_tty = True
            self._make_raw(fd)
            return
        raise RuntimeError(
            'No USB keyboard and this process has no terminal.\n'
            'Keep HW running, then in a second SSH terminal:\n'
            '  source ~/skill23_ws/install/setup.bash\n'
            '  ros2 run mypackage oms_node')

    def _make_raw(self, fd: int) -> None:
        try:
            self._tty_settings = termios.tcgetattr(fd)
            updated = termios.tcgetattr(fd)
            updated[3] &= ~(termios.ECHO | termios.ICANON)
            updated[6][termios.VMIN] = 0
            updated[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSADRAIN, updated)
            try:
                flags = fcntl.fcntl(fd, fcntl.F_GETFL)
                fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
            except OSError:
                pass
        except termios.error:
            self._tty_settings = None

    def _read_tty(self) -> None:
        if not self.running or self.fd < 0:
            return
        try:
            data = os.read(self.fd, 32)
        except BlockingIOError:
            return
        except OSError as exc:
            if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                return
            if exc.errno in (errno.ENODEV, errno.EBADF, errno.EIO):
                self.get_logger().error('Terminal closed; stopping OMS')
                self._stop_continuous('disconnect')
                self.running = False
                rclpy.shutdown()
            return
        if not data:
            return
        text = data.decode('latin-1', errors='ignore')
        i = 0
        while i < len(text):
            ch = text[i]
            if ch == '\x1b':
                if i + 1 < len(text):
                    i += 3 if text[i + 1] == '[' else 2
                    continue
                self._handle_key(KEY_ESC, 1)
                i += 1
                continue
            code = CHAR_TO_CODE.get(ch)
            if code is not None:
                self._handle_key(code, 1)
            i += 1

    def _read_events(self) -> None:
        while self.running:
            try:
                data = os.read(self.fd, INPUT_EVENT.size * 32)
            except BlockingIOError:
                return
            except OSError as exc:
                if exc.errno in (errno.ENODEV, errno.EBADF):
                    self.get_logger().error('Keyboard disconnected; stopping OMS')
                    self._stop_continuous('disconnect')
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
            self._titan(self.rotate_motor, -self.rotate_duty, 'rotate ACW')
        elif code == KEY_E:
            self._titan(self.rotate_motor, self.rotate_duty, 'rotate CW')
        elif code == KEY_R:
            self._titan(self.lift_motor, -self.lift_duty, 'lift up')
        elif code == KEY_F:
            self._titan(self.lift_motor, self.lift_duty, 'lift down')
        elif code == KEY_T:
            self._linear(-self.linear_speed, 'linear forward')
        elif code == KEY_G:
            self._linear(self.linear_speed, 'linear backward')
        elif code == KEY_Y:
            self._servo(self.vertical_cli, self.vertical_up, 'vertical up')
        elif code == KEY_H:
            self._servo(self.vertical_cli, self.vertical_down, 'vertical down')
        elif code == KEY_U:
            self._servo(self.rotation_cli, self.rotation_in, 'rotation in')
        elif code == KEY_J:
            self._servo(self.rotation_cli, self.rotation_out, 'rotation out')
        elif code == KEY_I:
            self._servo(self.gripper_cli, self.gripper_open, 'gripper open')
        elif code == KEY_K:
            self._servo(self.gripper_cli, self.gripper_close, 'gripper close')
        elif code == KEY_SPACE:
            self._stop_continuous('SPACE')
        elif code == KEY_ESC:
            self.get_logger().info('ESC: quitting (SPACE only stops motors)')
            self._stop_continuous('quit')
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

    def _linear(self, speed: float, label: str) -> None:
        """One cmd on /linear_servo/cmd. Continuous PWM holds until the next cmd.

        Do not stream the same speed — each SetAngle can glitch the pulse.
        Speed is an int in −100…100; values near 0 are in the servo deadband.
        """
        cmd = max(-100, min(100, int(round(speed))))
        if cmd == self._linear_cmd:
            return
        self._linear_cmd = cmd
        msg = Float64()
        msg.data = float(cmd)
        self._linear_pub.publish(msg)
        self.get_logger().info(f'{label} → {cmd}  (/linear_servo/cmd)')

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

    def _stop_continuous(self, reason: str = 'stop') -> None:
        self._titan(self.rotate_motor, 0.0, 'rotate stop')
        self._titan(self.lift_motor, 0.0, 'lift stop')
        self._linear(0.0, 'linear stop')
        self.get_logger().info(
            f'{reason}: motors+linear set to 0 (node still running)'
            if reason == 'SPACE' else f'{reason}: motors+linear set to 0')

    def close(self) -> None:
        if rclpy.ok():
            self._stop_continuous('shutdown')
        if self._tty_settings is not None and self.fd >= 0:
            try:
                termios.tcsetattr(self.fd, termios.TCSADRAIN, self._tty_settings)
            except termios.error:
                pass
            self._tty_settings = None
        if self._own_fd and self.fd >= 0:
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
