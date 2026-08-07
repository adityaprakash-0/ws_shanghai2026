#!/usr/bin/env python3
"""Relative move: X then Y then yaw — for fine station adjustments."""

from __future__ import annotations

import json
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


def normalize(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_of(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)


class RelativeMoveNode(Node):
    def __init__(self):
        super().__init__('pp_relative_move')
        self.declare_parameter('odom_topic', '/pp/odom')
        self.declare_parameter('cmd_vel_topic', '/pp/cmd_vel')
        self.declare_parameter('robot', 'mecanum')
        self.declare_parameter('vmax', 0.18)
        self.declare_parameter('wmax', 0.45)
        self.declare_parameter('xy_tol', 0.015)
        self.declare_parameter('yaw_tol', 0.05)
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('timeout_sec', 15.0)

        self._robot = self.get_parameter('robot').value
        self._vmax = float(self.get_parameter('vmax').value)
        self._wmax = float(self.get_parameter('wmax').value)
        self._xy_tol = float(self.get_parameter('xy_tol').value)
        self._yaw_tol = float(self.get_parameter('yaw_tol').value)
        self._kp = float(self.get_parameter('kp').value)
        self._timeout = float(self.get_parameter('timeout_sec').value)

        self._odom = None
        self._lock = threading.Lock()
        self._busy = False
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self._on_odom, 10)
        self._cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self._result_pub = self.create_publisher(String, '/pp/nav/result', 10)
        self.create_subscription(String, '/pp/nav/relative', self._on_cmd, 10)

    def _on_odom(self, msg: Odometry):
        self._odom = msg

    def _stop(self):
        self._cmd_pub.publish(Twist())

    def _result(self, cmd_id, ok, message):
        msg = String()
        msg.data = json.dumps({'cmd_id': cmd_id, 'success': ok, 'message': message, 'type': 'relative'})
        self._result_pub.publish(msg)

    def _on_cmd(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except Exception as exc:
            self._result('', False, str(exc))
            return
        threading.Thread(target=self._run, args=(cmd,), daemon=True).start()

    def _wait_odom(self):
        t0 = time.time()
        while rclpy.ok() and self._odom is None and time.time() - t0 < 5.0:
            time.sleep(0.05)
        return self._odom is not None

    def _axis_move(self, axis: str, delta: float) -> bool:
        if abs(delta) < 1e-4:
            return True
        start = self._odom
        sx, sy, syaw = start.pose.pose.position.x, start.pose.pose.position.y, yaw_of(start)
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < self._timeout:
            ox = self._odom.pose.pose.position.x
            oy = self._odom.pose.pose.position.y
            oyaw = yaw_of(self._odom)
            # Progress in start body frame
            c, s = math.cos(syaw), math.sin(syaw)
            dx = ox - sx
            dy = oy - sy
            bx = c * dx + s * dy
            by = -s * dx + c * dy
            if axis == 'x':
                err = delta - bx
                done = abs(err) <= self._xy_tol
                vx, vy, wz = self._kp * err, 0.0, 0.0
            elif axis == 'y':
                err = delta - by
                done = abs(err) <= self._xy_tol
                vx, vy, wz = 0.0, self._kp * err, 0.0
                if self._robot in ('allterrain', 'grip'):
                    # Diff drive cannot strafe — rotate 90, drive, rotate back handled by caller
                    return False
            else:
                err = normalize(delta - normalize(oyaw - syaw))
                done = abs(err) <= self._yaw_tol
                vx, vy, wz = 0.0, 0.0, self._kp * err

            if done:
                self._stop()
                time.sleep(0.15)
                return True

            vx = max(-self._vmax, min(self._vmax, vx))
            vy = max(-self._vmax, min(self._vmax, vy))
            wz = max(-self._wmax, min(self._wmax, wz))
            tw = Twist()
            tw.linear.x = float(vx)
            tw.linear.y = float(vy)
            tw.angular.z = float(wz)
            self._cmd_pub.publish(tw)
            time.sleep(0.05)
        self._stop()
        return False

    def _run(self, cmd: dict):
        with self._lock:
            if self._busy:
                self._result(str(cmd.get('cmd_id', '')), False, 'busy')
                return
            self._busy = True
        cmd_id = str(cmd.get('cmd_id', ''))
        ok = False
        message = 'failed'
        try:
            if not self._wait_odom():
                self._result(cmd_id, False, 'no odom')
                return
            dx = float(cmd.get('dx', cmd.get('x', 0.0)))
            dy = float(cmd.get('dy', cmd.get('y', 0.0)))
            dyaw = float(cmd.get('dyaw', cmd.get('yaw', 0.0)))
            if not self._axis_move('x', dx):
                message = 'x failed'
            elif self._robot not in ('allterrain', 'grip') and not self._axis_move('y', dy):
                message = 'y failed'
            elif self._robot in ('allterrain', 'grip') and abs(dy) > 1e-4:
                message = 'diff-drive cannot strafe (dy ignored)'
                # still continue to yaw
                if not self._axis_move('yaw', dyaw):
                    message = 'yaw failed'
                else:
                    ok, message = True, 'done (dy skipped)'
            elif not self._axis_move('yaw', dyaw):
                message = 'yaw failed'
            else:
                ok, message = True, 'done'
        finally:
            self._stop()
            with self._lock:
                self._busy = False
            self._result(cmd_id, ok, message)


def main(args=None):
    rclpy.init(args=args)
    node = RelativeMoveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
