#!/usr/bin/env python3
"""Mecanum task manager — waits for physical START, then runs mission sequence.

Hardware start_switch (DIO pin 8) publishes /start_switch/state (std_msgs/Bool):
  unpressed → True
  pressed   → False

This node INVERTS that so:
  unpressed → pressed_logical False
  pressed   → pressed_logical True

Sequence starts on rising edge of pressed_logical (button press).
When the last step finishes → solid red on /light_tower/set and MISSION COMPLETE.
Moving/stopped colors during the run come from light_tower follow_cmd_vel
watching /cmd_vel (in params_mecanum_task.yaml).
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

try:
    from studica_control.srv import SetData
except ImportError:  # pragma: no cover
    SetData = None


# ----- timings you can change -----
START_SETTLE_SEC = 0.5   # delay after START press before first task
BETWEEN_STEPS_SEC = 1.0  # wait after each move/turn done before next task


class TaskManager(Node):
    def __init__(self) -> None:
        super().__init__('task_manager_mecanum')

        self.declare_parameter('start_topic', '/start_switch/state')
        self.declare_parameter('invert_start', True)  # HW True=unpressed
        self.declare_parameter('light_service', '/light_tower/set')

        start_topic = str(self.get_parameter('start_topic').value)
        self._invert = bool(self.get_parameter('invert_start').value)
        light_srv = str(self.get_parameter('light_service').value)

        self.move_pub = self.create_publisher(Point, '/pid_move/goal', 10)
        self.turn_pub = self.create_publisher(Float32, '/pid_turn/goal', 10)

        self.create_subscription(Bool, '/pid_move/done', self._on_action_done, 10)
        self.create_subscription(Bool, '/pid_turn/done', self._on_action_done, 10)
        self.create_subscription(Bool, start_topic, self._on_start_switch, 10)

        self._light_service = light_srv
        self._light_cli = None
        if SetData is not None:
            self._light_cli = self.create_client(SetData, light_srv)

        # Format: ('move', distance_m, axis_mode, heading_lock_deg)
        #   axis_mode: 0.0 = Forward/Back, 1.0 = Strafe
        # Format: ('turn', angle_deg)
        self.mission_plan = [
            ('move', 1.0, 0.0, 0.0),   # FORWARD 1 m
            # ('turn', -90.0),
            # ('move', 0.5, 1.0, 0.0),  # STRAFE 0.5 m
            # ('turn', 90.0),
        ]

        self.current_step = 0
        self._started = False
        self._busy = False
        self._finished = False
        self._prev_pressed = False
        self._raw_hw = None
        self._timer = None

        self.get_logger().info(
            'Mecanum Task Manager ready — waiting for START button '
            f'(topic={start_topic}, invert={self._invert}, '
            f'DIO raw: unpressed=True pressed=False)')
        self.get_logger().info(
            f'Mission has {len(self.mission_plan)} step(s). Press start_switch to begin.')

    # ----- start switch (inverted) -----

    def _on_start_switch(self, msg: Bool) -> None:
        raw = bool(msg.data)
        self._raw_hw = raw
        # Invert: pressed_logical True when button is physically pressed
        pressed = (not raw) if self._invert else raw

        if pressed != self._prev_pressed:
            self.get_logger().info(
                f'start_switch: hw_raw={raw}  pressed_logical={pressed}')

        # Rising edge of logical press → start once
        if pressed and not self._prev_pressed:
            if not self._started and not self._finished:
                self.get_logger().info(
                    '>>> START BUTTON PRESSED — beginning mission sequence')
                self._started = True
                self.current_step = 0
                self._busy = False
                # short settle then first task
                if self._timer is not None:
                    self._timer.cancel()
                self._timer = self.create_timer(START_SETTLE_SEC, self._start_next_task)
            elif self._finished:
                self.get_logger().info(
                    'START pressed but mission already finished — restart launch to run again')

        self._prev_pressed = pressed

    # ----- sequence -----

    def _on_action_done(self, msg: Bool) -> None:
        if not msg.data or not self._started or self._finished:
            return
        if not self._busy:
            return
        self._busy = False
        self.get_logger().info(
            f'Action DONE (step {self.current_step + 1}/'
            f'{len(self.mission_plan)}). Next in {BETWEEN_STEPS_SEC} s…')
        self.current_step += 1
        if self._timer is not None:
            self._timer.cancel()
        self._timer = self.create_timer(BETWEEN_STEPS_SEC, self._start_next_task)

    def _start_next_task(self) -> None:
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

        if self.current_step >= len(self.mission_plan):
            self._mission_complete()
            return

        task = self.mission_plan[self.current_step]
        action_type = task[0]
        self._busy = True

        if action_type == 'move':
            goal = Point()
            goal.x = float(task[1])
            goal.y = float(task[2])
            goal.z = float(task[3])
            axis_str = 'Forward/Back' if goal.y == 0.0 else 'Strafing'
            self.get_logger().info(
                f'Task {self.current_step + 1}/{len(self.mission_plan)}: '
                f'MOVE {goal.x} m ({axis_str}), heading_lock={goal.z}')
            self.move_pub.publish(goal)

        elif action_type == 'turn':
            goal = Float32()
            goal.data = float(task[1])
            self.get_logger().info(
                f'Task {self.current_step + 1}/{len(self.mission_plan)}: '
                f'TURN {goal.data} deg')
            self.turn_pub.publish(goal)
        else:
            self.get_logger().error(f'Unknown task type: {action_type}')
            self._busy = False
            self.current_step += 1
            self._timer = self.create_timer(0.2, self._start_next_task)

    def _mission_complete(self) -> None:
        self._finished = True
        self._busy = False
        self.get_logger().info('========== MISSION COMPLETE ==========')
        self.get_logger().info('Sending solid RED to light tower (finish)')
        self._set_light('red')
        print('========== MISSION COMPLETE — light tower SOLID RED ==========',
              flush=True)

    def _set_light(self, color: str) -> None:
        if self._light_cli is None:
            self.get_logger().warn('SetData / light_tower unavailable — skip light')
            return
        if not self._light_cli.service_is_ready():
            self.get_logger().warn(
                f'{self._light_service} not ready — cannot set {color}')
            return
        req = SetData.Request()
        req.params = color  # solid (no :blink)
        self._light_cli.call_async(req)
        self.get_logger().info(f'light_tower → {color}:solid')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaskManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
