#!/usr/bin/env python3
"""Elevator (Titan1 m0) + linear/gripper standard servos for pick-place."""

from __future__ import annotations

import json
import math
import threading
import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String


class ManipulatorNode(Node):
    def __init__(self):
        super().__init__('pp_manipulator')
        self.declare_parameter('elevator_cmd_topic', '/titan1/m_0/cmd')
        self.declare_parameter('elevator_enc_topic', '/titan1/m_0/encoder')
        self.declare_parameter('linear_cmd_topic', '/linear_servo/cmd')
        self.declare_parameter('gripper_cmd_topic', '/gripper_servo/cmd')
        self.declare_parameter('elevator_duty', 0.45)
        self.declare_parameter('elevator_slow_duty', 0.20)
        self.declare_parameter('elevator_deadband_ticks', 40.0)
        self.declare_parameter('elevator_slow_zone_ticks', 200.0)
        self.declare_parameter('elevator_timeout_sec', 12.0)
        self.declare_parameter('elevator_progress_timeout_sec', 2.0)
        self.declare_parameter('servo_settle_sec', 0.8)

        self._elev_duty = float(self.get_parameter('elevator_duty').value)
        self._elev_slow = float(self.get_parameter('elevator_slow_duty').value)
        self._deadband = float(self.get_parameter('elevator_deadband_ticks').value)
        self._slow_zone = float(self.get_parameter('elevator_slow_zone_ticks').value)
        self._timeout = float(self.get_parameter('elevator_timeout_sec').value)
        self._progress_timeout = float(self.get_parameter('elevator_progress_timeout_sec').value)
        self._servo_settle = float(self.get_parameter('servo_settle_sec').value)

        # Named targets — declare as JSON strings for easy YAML override
        self.declare_parameter(
            'elevator_targets_json',
            json.dumps({'up': 0.0, 'up_mid': 700.0, 'down_mid': 1400.0, 'down': 2000.0}))
        self.declare_parameter(
            'linear_angles_json',
            json.dumps({'retract': 0.0, 'mid': 45.0, 'extend': 90.0}))
        self.declare_parameter(
            'gripper_angles_json',
            json.dumps({'open': 90.0, 'close': -20.0}))

        self._elev_targets: Dict[str, float] = json.loads(
            self.get_parameter('elevator_targets_json').value)
        self._linear_angles: Dict[str, float] = json.loads(
            self.get_parameter('linear_angles_json').value)
        self._gripper_angles: Dict[str, float] = json.loads(
            self.get_parameter('gripper_angles_json').value)

        elev_cmd = self.get_parameter('elevator_cmd_topic').value
        elev_enc = self.get_parameter('elevator_enc_topic').value
        lin_cmd = self.get_parameter('linear_cmd_topic').value
        grip_cmd = self.get_parameter('gripper_cmd_topic').value

        self._elev_pub = self.create_publisher(Float64, elev_cmd, 10)
        self._linear_pub = self.create_publisher(Float64, lin_cmd, 10)
        self._gripper_pub = self.create_publisher(Float64, grip_cmd, 10)
        self._result_pub = self.create_publisher(String, '/pp/manipulator/result', 10)
        self._status_pub = self.create_publisher(String, '/pp/manipulator/status', 10)

        self._enc = 0.0
        self.create_subscription(Float64, elev_enc, self._on_enc, 10)
        self.create_subscription(String, '/pp/manipulator/cmd', self._on_cmd, 10)

        self._lock = threading.Lock()
        self._busy = False
        self._last_cmd_id = ''
        self.create_timer(0.5, self._publish_status)
        self.get_logger().info(
            'Manipulator ready — elevator Titan1/m0, linear+gripper servos')

    def _on_enc(self, msg: Float64):
        self._enc = float(msg.data)

    def _publish_status(self):
        msg = String()
        msg.data = json.dumps({
            'busy': self._busy,
            'encoder': round(self._enc, 1),
            'last_cmd_id': self._last_cmd_id,
        })
        self._status_pub.publish(msg)

    def _publish_result(self, cmd_id: str, ok: bool, message: str):
        msg = String()
        msg.data = json.dumps({
            'cmd_id': cmd_id,
            'success': ok,
            'message': message,
            'encoder': self._enc,
        })
        self._result_pub.publish(msg)

    def _stop_elevator(self):
        out = Float64()
        out.data = 0.0
        self._elev_pub.publish(out)

    def _set_servo(self, pub, angle: float):
        out = Float64()
        out.data = float(angle)
        pub.publish(out)

    def _move_elevator_to(self, target: float) -> tuple[bool, str]:
        start = time.time()
        last_progress = time.time()
        last_enc = self._enc
        while rclpy.ok():
            err = target - self._enc
            if abs(err) <= self._deadband:
                self._stop_elevator()
                return True, f'elevator reached {self._enc:.0f}'
            if time.time() - start > self._timeout:
                self._stop_elevator()
                return False, 'elevator timeout'
            if abs(self._enc - last_enc) > 15.0:
                last_enc = self._enc
                last_progress = time.time()
            elif time.time() - last_progress > self._progress_timeout:
                self._stop_elevator()
                return False, 'elevator no progress'

            duty = self._elev_slow if abs(err) < self._slow_zone else self._elev_duty
            out = Float64()
            out.data = duty if err > 0 else -duty
            self._elev_pub.publish(out)
            time.sleep(0.02)
        self._stop_elevator()
        return False, 'shutdown'

    def _resolve_elev(self, value) -> Optional[float]:
        if isinstance(value, (int, float)):
            return float(value)
        key = str(value).strip().lower().replace('-', '_')
        aliases = {
            'up': 'up', 'up_mid': 'up_mid', 'upmid': 'up_mid',
            'down_mid': 'down_mid', 'downmid': 'down_mid',
            'down': 'down',
        }
        key = aliases.get(key, key)
        return self._elev_targets.get(key)

    def _resolve_linear(self, value) -> Optional[float]:
        if isinstance(value, (int, float)):
            return float(value)
        key = str(value).strip().lower()
        return self._linear_angles.get(key)

    def _resolve_gripper(self, value) -> Optional[float]:
        if isinstance(value, (int, float)):
            return float(value)
        key = str(value).strip().lower()
        return self._gripper_angles.get(key)

    def _on_cmd(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except Exception as exc:
            self._publish_result('', False, f'bad json: {exc}')
            return
        threading.Thread(target=self._run_cmd, args=(cmd,), daemon=True).start()

    def _run_cmd(self, cmd: dict):
        with self._lock:
            if self._busy:
                self._publish_result(str(cmd.get('cmd_id', '')), False, 'busy')
                return
            self._busy = True
        cmd_id = str(cmd.get('cmd_id', f'cmd_{int(time.time()*1000)}'))
        self._last_cmd_id = cmd_id
        action = str(cmd.get('action', 'sequence')).lower()
        ok = True
        message = 'ok'
        try:
            if action in ('pick', 'place', 'sequence', 'move'):
                elev = cmd.get('elevator')
                if elev is not None:
                    target = self._resolve_elev(elev)
                    if target is None:
                        ok, message = False, f'unknown elevator target {elev}'
                    else:
                        ok, message = self._move_elevator_to(target)
                if ok and cmd.get('extend') is not None:
                    ang = self._resolve_linear(cmd.get('extend'))
                    if ang is None:
                        ok, message = False, f'unknown linear {cmd.get("extend")}'
                    else:
                        self._set_servo(self._linear_pub, ang)
                        time.sleep(self._servo_settle)
                if ok and cmd.get('gripper') is not None:
                    ang = self._resolve_gripper(cmd.get('gripper'))
                    if ang is None:
                        ok, message = False, f'unknown gripper {cmd.get("gripper")}'
                    else:
                        self._set_servo(self._gripper_pub, ang)
                        time.sleep(self._servo_settle)
            elif action == 'elevator':
                target = self._resolve_elev(cmd.get('elevator', 'up'))
                if target is None:
                    ok, message = False, 'bad elevator'
                else:
                    ok, message = self._move_elevator_to(target)
            elif action == 'linear':
                ang = self._resolve_linear(cmd.get('extend', 'retract'))
                if ang is None:
                    ok, message = False, 'bad linear'
                else:
                    self._set_servo(self._linear_pub, ang)
                    time.sleep(self._servo_settle)
            elif action == 'gripper':
                ang = self._resolve_gripper(cmd.get('gripper', 'open'))
                if ang is None:
                    ok, message = False, 'bad gripper'
                else:
                    self._set_servo(self._gripper_pub, ang)
                    time.sleep(self._servo_settle)
            elif action == 'stop':
                self._stop_elevator()
            else:
                ok, message = False, f'unknown action {action}'
        except Exception as exc:
            ok, message = False, str(exc)
            self._stop_elevator()
        finally:
            with self._lock:
                self._busy = False
            self._publish_result(cmd_id, ok, message)

    def destroy_node(self):
        self._stop_elevator()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ManipulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
