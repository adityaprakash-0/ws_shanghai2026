#!/usr/bin/env python3
"""Mecanum colore mission — ask colors → wait START → run sequence with mid-move detect.

Typical sequence steps:
  ('move', dist_m, axis, heading)   axis 0=fwd/back  1=strafe
  ('turn', deg)
  ('colore_on',)   # color name asked in terminal before start (slot order)
  ('colore_off',)

While colore is ON, a forward/strafe move watches /colore_detect/found.
On hit: cancel PID → buzzer 3s → green flash 2s → watch off → finish remaining distance.

Run AFTER HW launch (other terminal):
  ros2 run mypackage task_manager_mecanum_colore
"""

from __future__ import annotations

import sys
from typing import List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

try:
    from studica_control.srv import SetData
except ImportError:  # pragma: no cover
    SetData = None

from mypackage.colore_detect_node import ALLOWED

# timings
START_SETTLE_SEC = 0.5
BETWEEN_STEPS_SEC = 1.0
BUZZER_SEC = 3.0
GREEN_FLASH_SEC = 2.0
REMAIN_TOL_M = 0.03

# Edit this plan — each ('colore_on',) gets one prompted color name (in order).
DEFAULT_MISSION_PLAN: List[Tuple] = [
    ('move', 0.50, 1.0, 0.0),   # strafe 50 cm
    ('colore_on',),
    ('move', 2.00, 0.0, 0.0),   # forward 200 cm (watch color)
    ('colore_off',),
    ('move', 0.70, 1.0, 0.0),   # strafe 70 cm
    ('colore_on',),
    ('move', 1.00, 0.0, 0.0),   # forward 100 cm (watch next color)
    ('colore_off',),
]


class TaskManagerMecanumColore(Node):
    def __init__(
        self,
        color_slots: List[str],
        mission_plan: Optional[Sequence[Tuple]] = None,
    ) -> None:
        super().__init__('task_manager_mecanum_colore')

        self.declare_parameter('start_topic', '/start_switch/state')
        self.declare_parameter('invert_start', True)
        self.declare_parameter('light_service', '/light_tower/set')

        start_topic = str(self.get_parameter('start_topic').value)
        self._invert = bool(self.get_parameter('invert_start').value)
        light_srv = str(self.get_parameter('light_service').value)

        self._color_slots = list(color_slots)  # one per colore_on in plan
        self._slot_i = 0

        self.move_pub = self.create_publisher(Point, '/pid_move/goal', 10)
        self.turn_pub = self.create_publisher(Float32, '/pid_turn/goal', 10)
        self.cancel_pub = self.create_publisher(Bool, '/pid_move/cancel', 10)
        self.watch_pub = self.create_publisher(String, '/colore_detect/watch', 10)

        self.create_subscription(Bool, '/pid_move/done', self._on_move_done, 10)
        self.create_subscription(Bool, '/pid_turn/done', self._on_turn_done, 10)
        self.create_subscription(Bool, '/colore_detect/found', self._on_color_found, 10)
        self.create_subscription(Float32, '/robot_distance_x', self._on_dist_x, 10)
        self.create_subscription(Float32, '/robot_distance_y', self._on_dist_y, 10)
        self.create_subscription(Bool, start_topic, self._on_start_switch, 10)

        self._light_service = light_srv
        self._light_cli = None
        if SetData is not None:
            self._light_cli = self.create_client(SetData, light_srv)

        self.mission_plan = list(mission_plan if mission_plan is not None else DEFAULT_MISSION_PLAN)

        n_on = sum(1 for t in self.mission_plan if t[0] == 'colore_on')
        if len(self._color_slots) != n_on:
            raise ValueError(
                f'need {n_on} color names for colore_on steps, got {len(self._color_slots)}')

        self.current_step = 0
        self._started = False
        self._busy = False
        self._finished = False
        self._prev_pressed = False
        self._timer = None

        self._colore_active = False
        self._active_color: Optional[str] = None
        self._move_watching = False
        self._handling_hit = False
        self._continuing_remain = False

        self._goal_dist = 0.0
        self._goal_axis = 0.0
        self._goal_heading = 0.0
        self._dist_x = 0.0
        self._dist_y = 0.0
        self._pending_remain = 0.0
        self._pending_axis = 0.0
        self._pending_heading = 0.0

        self.get_logger().info(
            f'Colore mission ready — colors={self._color_slots}  '
            f'steps={len(self.mission_plan)}. Press START (DIO 8) to begin.')

    # ----- feedback -----

    def _on_dist_x(self, msg: Float32) -> None:
        self._dist_x = float(msg.data)

    def _on_dist_y(self, msg: Float32) -> None:
        self._dist_y = float(msg.data)

    def _traveled(self) -> float:
        return self._dist_x if self._goal_axis == 0.0 else self._dist_y

    # ----- start -----

    def _on_start_switch(self, msg: Bool) -> None:
        raw = bool(msg.data)
        pressed = (not raw) if self._invert else raw
        if pressed and not self._prev_pressed:
            if not self._started and not self._finished:
                self.get_logger().info('>>> START PRESSED — beginning colore mission')
                print('>>> START PRESSED — beginning colore mission', flush=True)
                self._started = True
                self.current_step = 0
                self._slot_i = 0
                self._busy = False
                if self._timer:
                    self._timer.cancel()
                self._timer = self.create_timer(START_SETTLE_SEC, self._start_next_task)
            elif self._finished:
                self.get_logger().info('Mission already finished — relaunch node to run again')
        self._prev_pressed = pressed

    # ----- sequence -----

    def _on_move_done(self, msg: Bool) -> None:
        if not msg.data or not self._started or self._finished:
            return
        if self._handling_hit:
            return
        if not self._busy and not self._continuing_remain:
            return
        self.get_logger().info(
            f'MOVE DONE (step {self.current_step + 1}) traveled≈{self._traveled():.3f} m')
        self._move_watching = False
        self._continuing_remain = False
        self._busy = False
        self.current_step += 1
        self._schedule_next(BETWEEN_STEPS_SEC)

    def _on_turn_done(self, msg: Bool) -> None:
        if not msg.data or not self._started or self._finished or not self._busy:
            return
        if self._handling_hit:
            return
        self.get_logger().info(f'TURN DONE (step {self.current_step + 1})')
        self._busy = False
        self.current_step += 1
        self._schedule_next(BETWEEN_STEPS_SEC)

    def _schedule_next(self, delay: float) -> None:
        if self._timer:
            self._timer.cancel()
        self._timer = self.create_timer(delay, self._start_next_task)

    def _start_next_task(self) -> None:
        if self._timer:
            self._timer.cancel()
            self._timer = None
        if self._handling_hit:
            return
        if self.current_step >= len(self.mission_plan):
            self._mission_complete()
            return

        task = self.mission_plan[self.current_step]
        kind = task[0]

        if kind == 'colore_on':
            color = self._color_slots[self._slot_i]
            self._slot_i += 1
            self._colore_active = True
            self._active_color = color
            w = String()
            w.data = color
            self.watch_pub.publish(w)
            self.get_logger().info(f'Task {self.current_step + 1}: COLORE ON → watch "{color}"')
            print(f'COLORE ON — watching "{color}"', flush=True)
            self.current_step += 1
            self._schedule_next(0.2)
            return

        if kind == 'colore_off':
            self._set_watch_off()
            self.get_logger().info(f'Task {self.current_step + 1}: COLORE OFF')
            print('COLORE OFF', flush=True)
            self.current_step += 1
            self._schedule_next(0.2)
            return

        if kind == 'move':
            self._busy = True
            self._goal_dist = float(task[1])
            self._goal_axis = float(task[2])
            self._goal_heading = float(task[3])
            self._move_watching = self._colore_active
            goal = Point()
            goal.x = self._goal_dist
            goal.y = self._goal_axis
            goal.z = self._goal_heading
            axis = 'strafe' if goal.y == 1.0 else 'fwd/back'
            watch = f' [watching {self._active_color}]' if self._move_watching else ''
            self.get_logger().info(
                f'Task {self.current_step + 1}: MOVE {goal.x} m ({axis}){watch}')
            print(f'MOVE {goal.x} m ({axis}){watch}', flush=True)
            self.move_pub.publish(goal)
            return

        if kind == 'turn':
            self._busy = True
            g = Float32()
            g.data = float(task[1])
            self.get_logger().info(f'Task {self.current_step + 1}: TURN {g.data} deg')
            self.turn_pub.publish(g)
            return

        self.get_logger().error(f'Unknown task {task}')
        self.current_step += 1
        self._schedule_next(0.2)

    # ----- color hit -----

    def _on_color_found(self, msg: Bool) -> None:
        if not msg.data or not self._move_watching or self._handling_hit:
            return
        if not self._busy:
            return
        traveled = self._traveled()
        remain = self._goal_dist - traveled
        self.get_logger().info(
            f'!!! COLOR "{self._active_color}" FOUND at ~{traveled:.3f} m '
            f'(goal {self._goal_dist:.3f} → remain {remain:.3f} m)')
        print(
            f'!!! COLOR FOUND "{self._active_color}" at {traveled:.3f} m — '
            f'stopping, then finish remaining {remain:.3f} m',
            flush=True)

        self._handling_hit = True
        self._move_watching = False
        self._busy = False

        # Cancel active PID move (no /done published)
        c = Bool()
        c.data = True
        self.cancel_pub.publish(c)

        self._pending_remain = remain
        self._pending_axis = self._goal_axis
        self._pending_heading = self._goal_heading
        self._set_watch_off()
        self._set_light('buzzer')
        self.get_logger().info(f'Buzzer ON for {BUZZER_SEC}s')
        if self._timer:
            self._timer.cancel()
        self._timer = self.create_timer(BUZZER_SEC, self._after_buzzer)

    def _after_buzzer(self) -> None:
        if self._timer:
            self._timer.cancel()
            self._timer = None
        self._set_light('off')
        self._set_light('green:2.0')  # flash green ~2 Hz
        self.get_logger().info(f'Green flash for {GREEN_FLASH_SEC}s')
        self._timer = self.create_timer(GREEN_FLASH_SEC, self._after_green)

    def _after_green(self) -> None:
        if self._timer:
            self._timer.cancel()
            self._timer = None
        self._set_light('off')
        remain = self._pending_remain
        axis = self._pending_axis
        heading = self._pending_heading
        self._handling_hit = False

        if abs(remain) > REMAIN_TOL_M:
            self.get_logger().info(f'Continuing remaining {remain:.3f} m (no color watch)')
            print(f'Continuing remaining {remain:.3f} m', flush=True)
            self._continuing_remain = True
            self._busy = True
            self._move_watching = False
            self._goal_dist = remain
            self._goal_axis = axis
            self._goal_heading = heading
            goal = Point()
            goal.x = remain
            goal.y = axis
            goal.z = heading
            self.move_pub.publish(goal)
        else:
            self.get_logger().info('Remaining distance negligible — advance sequence')
            self.current_step += 1
            self._schedule_next(BETWEEN_STEPS_SEC)

    def _set_watch_off(self) -> None:
        self._colore_active = False
        self._active_color = None
        self._move_watching = False
        w = String()
        w.data = ''
        self.watch_pub.publish(w)

    def _mission_complete(self) -> None:
        self._finished = True
        self._set_watch_off()
        self.get_logger().info('========== MISSION COMPLETE ==========')
        print('========== MISSION COMPLETE ==========', flush=True)
        self._set_light('red')

    def _set_light(self, params: str) -> None:
        if self._light_cli is None:
            return
        if not self._light_cli.service_is_ready():
            self.get_logger().warn(f'{self._light_service} not ready ({params})')
            return
        req = SetData.Request()
        req.params = params
        self._light_cli.call_async(req)
        self.get_logger().info(f'light_tower → {params}')


def _prompt_colors(n: int) -> List[str]:
    print('\n=== Mecanum Colore Mission — assign colors ===', flush=True)
    print(f'Allowed: {", ".join(ALLOWED)}', flush=True)
    print(f'This mission has {n} colore_on segment(s).\n', flush=True)
    out: List[str] = []
    for i in range(n):
        while True:
            raw = input(f'Color name for colore_on #{i + 1}: ').strip().lower()
            if raw in ALLOWED:
                out.append(raw)
                print(f'  → slot {i + 1} = {raw}', flush=True)
                break
            print(f'  invalid — choose one of {ALLOWED}', flush=True)
    print('\nColors saved. Press the physical START button when ready.\n', flush=True)
    return out


def main(args=None) -> None:
    plan = list(DEFAULT_MISSION_PLAN)
    n_on = sum(1 for t in plan if t[0] == 'colore_on')
    try:
        slots = _prompt_colors(n_on)
    except EOFError:
        print('No interactive TTY — run in a real terminal.', file=sys.stderr)
        sys.exit(1)

    rclpy.init(args=args)
    node = TaskManagerMecanumColore(slots, mission_plan=plan)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
