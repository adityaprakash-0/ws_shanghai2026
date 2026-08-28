"""OMS click-to-run from a gamepad — same values and behaviour as oms_node.

Subscribes to /joy. Drive stays on studica gamepad (left stick / axis 2 / RB).
This node only uses leftover buttons / D-pad.

Click = one command. Rotate, lift, and linear keep running until Options (stop).

Default Linux DualShock 4 (PS4):
  D-pad L/R   rotate ACW / CW     titan1 m_2
  D-pad U/D   lift up / down      titan1 m_1
  L1 / Share  linear fwd / back   /linear_servo/cmd
  Options     stop rotate+lift+linear   ← this is “Start”
  Square / Triangle  rotation horiz / vert
  Cross / Circle     gripper open / close
  L3 / R3     vertical down / up

  ros2 launch mypackage oms_joy.launch.py enable_servos:=true
  # HW already up:
  ros2 launch mypackage oms_joy.launch.py start_hw:=false
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from studica_control.srv import SetData


ROTATE_MOTOR = 2
LIFT_MOTOR = 1


class OmsJoyNode(Node):
    def __init__(self) -> None:
        super().__init__('oms_joy_node')

        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('titan_service', '/titan1/titan_cmd')
        self.declare_parameter('linear_cmd_topic', '/linear_servo/cmd')
        self.declare_parameter('vertical_service', '/vertical_servo/set_servo')
        self.declare_parameter('rotation_service', '/rotation_servo/set_servo')
        self.declare_parameter('gripper_service', '/gripper_servo/set_servo')

        # Same tuned values as oms_node.
        self.declare_parameter('rotate_duty', 0.2)
        self.declare_parameter('lift_duty', 0.50)
        self.declare_parameter('rotate_motor', ROTATE_MOTOR)
        self.declare_parameter('lift_motor', LIFT_MOTOR)
        self.declare_parameter('linear_speed', 30.0)
        self.declare_parameter('vertical_up_deg', 50.0)
        self.declare_parameter('vertical_down_deg', -40.0)
        self.declare_parameter('rotation_horizontal_deg', 70.0)
        self.declare_parameter('rotation_vertical_deg', 170.0)
        self.declare_parameter('gripper_open_deg', 100.0)
        self.declare_parameter('gripper_close_deg', -100.0)
        self.declare_parameter('dpad_threshold', 0.5)

        # Axes: -1 = unused. Defaults avoid drive axes 1 and 2.
        self.declare_parameter('axis_dpad_x', 6)
        self.declare_parameter('axis_dpad_y', 7)

        # DualShock 4 (Linux hid-sony): avoid R1 (5) — drive turbo.
        self.declare_parameter('button_rotate_acw', -1)
        self.declare_parameter('button_rotate_cw', -1)
        self.declare_parameter('button_lift_up', -1)
        self.declare_parameter('button_lift_down', -1)
        self.declare_parameter('button_linear_fwd', 4)      # L1 = T
        self.declare_parameter('button_linear_back', 8)     # Share = G
        self.declare_parameter('button_stop', 9)            # Options = SPACE
        self.declare_parameter('button_vertical_up', 12)    # R3 = Y
        self.declare_parameter('button_vertical_down', 11)  # L3 = H
        self.declare_parameter('button_rotation_horiz', 3)  # Square = U
        self.declare_parameter('button_rotation_vert', 2)   # Triangle = J
        self.declare_parameter('button_gripper_open', 0)    # Cross = I
        self.declare_parameter('button_gripper_close', 1)   # Circle = K

        self.rotate_duty = float(self.get_parameter('rotate_duty').value)
        self.lift_duty = float(self.get_parameter('lift_duty').value)
        self.rotate_motor = int(self.get_parameter('rotate_motor').value)
        self.lift_motor = int(self.get_parameter('lift_motor').value)
        self.linear_speed = max(1.0, min(100.0, abs(float(
            self.get_parameter('linear_speed').value))))
        self.vertical_up = float(self.get_parameter('vertical_up_deg').value)
        self.vertical_down = float(self.get_parameter('vertical_down_deg').value)
        self.rotation_horizontal = float(
            self.get_parameter('rotation_horizontal_deg').value)
        self.rotation_vertical = float(
            self.get_parameter('rotation_vertical_deg').value)
        self.gripper_open = float(self.get_parameter('gripper_open_deg').value)
        self.gripper_close = float(self.get_parameter('gripper_close_deg').value)
        self.dpad_threshold = float(self.get_parameter('dpad_threshold').value)

        self.axis_dpad_x = int(self.get_parameter('axis_dpad_x').value)
        self.axis_dpad_y = int(self.get_parameter('axis_dpad_y').value)

        self.btn = {
            'rotate_acw': int(self.get_parameter('button_rotate_acw').value),
            'rotate_cw': int(self.get_parameter('button_rotate_cw').value),
            'lift_up': int(self.get_parameter('button_lift_up').value),
            'lift_down': int(self.get_parameter('button_lift_down').value),
            'linear_fwd': int(self.get_parameter('button_linear_fwd').value),
            'linear_back': int(self.get_parameter('button_linear_back').value),
            'stop': int(self.get_parameter('button_stop').value),
            'vertical_up': int(self.get_parameter('button_vertical_up').value),
            'vertical_down': int(self.get_parameter('button_vertical_down').value),
            'rotation_horiz': int(self.get_parameter('button_rotation_horiz').value),
            'rotation_vert': int(self.get_parameter('button_rotation_vert').value),
            'gripper_open': int(self.get_parameter('button_gripper_open').value),
            'gripper_close': int(self.get_parameter('button_gripper_close').value),
        }

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

        self._prev_buttons = []
        self._prev_dpad_x = 0
        self._prev_dpad_y = 0

        topic = str(self.get_parameter('joy_topic').value)
        self.create_subscription(Joy, topic, self._on_joy, 10)

        self.get_logger().info(
            f'OMS gamepad on {topic} (same values as oms_node)\n'
            f'  D-pad L/R  rotate ACW / CW   (titan1 m_{self.rotate_motor})\n'
            f'  D-pad U/D  lift up / down    (titan1 m_{self.lift_motor})\n'
            '  L1 / Share linear fwd / back (/linear_servo/cmd)\n'
            '  Square / Triangle  rotation horiz / vert\n'
            '  Cross / Circle     gripper open / close\n'
            '  L3 / R3    vertical down / up\n'
            '  Options    stop rotate+lift+linear (this is Start)\n'
            '  Left stick + R1(turbo) still drive the robot')

    def _on_joy(self, msg: Joy) -> None:
        buttons = list(msg.buttons)
        if len(self._prev_buttons) != len(buttons):
            self._prev_buttons = [0] * len(buttons)

        def pressed(name: str) -> bool:
            idx = self.btn[name]
            if idx < 0 or idx >= len(buttons):
                return False
            was = self._prev_buttons[idx] if idx < len(self._prev_buttons) else 0
            now = buttons[idx]
            return was == 0 and now == 1

        dpad_x = self._axis_dir(msg.axes, self.axis_dpad_x)
        dpad_y = self._axis_dir(msg.axes, self.axis_dpad_y)
        dpad_x_click = dpad_x if dpad_x != 0 and self._prev_dpad_x == 0 else 0
        dpad_y_click = dpad_y if dpad_y != 0 and self._prev_dpad_y == 0 else 0

        if pressed('stop'):
            self._stop_continuous('Options')
        elif dpad_x_click < 0 or pressed('rotate_acw'):
            self._titan(self.rotate_motor, -self.rotate_duty, 'rotate ACW')
        elif dpad_x_click > 0 or pressed('rotate_cw'):
            self._titan(self.rotate_motor, self.rotate_duty, 'rotate CW')
        elif dpad_y_click > 0 or pressed('lift_up'):
            self._titan(self.lift_motor, -self.lift_duty, 'lift up')
        elif dpad_y_click < 0 or pressed('lift_down'):
            self._titan(self.lift_motor, self.lift_duty, 'lift down')
        elif pressed('linear_fwd'):
            self._linear(-self.linear_speed, 'linear forward')
        elif pressed('linear_back'):
            self._linear(self.linear_speed, 'linear backward')
        elif pressed('vertical_up'):
            self._servo(self.vertical_cli, self.vertical_up, 'vertical up')
        elif pressed('vertical_down'):
            self._servo(self.vertical_cli, self.vertical_down, 'vertical down')
        elif pressed('rotation_horiz'):
            self._servo(self.rotation_cli, self.rotation_horizontal, 'rotation horizontal')
        elif pressed('rotation_vert'):
            self._servo(self.rotation_cli, self.rotation_vertical, 'rotation vertical')
        elif pressed('gripper_open'):
            self._servo(self.gripper_cli, self.gripper_open, 'gripper open')
        elif pressed('gripper_close'):
            self._servo(self.gripper_cli, self.gripper_close, 'gripper close')

        self._prev_buttons = buttons
        self._prev_dpad_x = dpad_x
        self._prev_dpad_y = dpad_y

    def _axis_dir(self, axes, idx: int) -> int:
        if idx < 0 or idx >= len(axes):
            return 0
        value = float(axes[idx])
        if value >= self.dpad_threshold:
            return 1
        if value <= -self.dpad_threshold:
            return -1
        return 0

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
        self.get_logger().info(f'{reason}: motors+linear set to 0 (node still running)')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = OmsJoyNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node._stop_continuous('shutdown')
            except Exception:
                pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
