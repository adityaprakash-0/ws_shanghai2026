#!/usr/bin/env python3
"""Absolute go-to-goal in /pp/odom frame — Team1-style PD, no Nav2."""

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


class GoToGoalNode(Node):
    def __init__(self):
        super().__init__('pp_go_to_goal')
        self.declare_parameter('odom_topic', '/pp/odom')
        self.declare_parameter('cmd_vel_topic', '/pp/cmd_vel')
        self.declare_parameter('robot', 'mecanum')
        self.declare_parameter('kp_lin', 0.8)
        self.declare_parameter('kd_lin', 0.02)
        self.declare_parameter('kp_ang', 1.6)
        self.declare_parameter('kd_ang', 0.02)
        self.declare_parameter('vmax', 0.22)
        self.declare_parameter('wmax', 0.55)
        self.declare_parameter('min_speed', 0.08)
        self.declare_parameter('min_ang_speed', 0.12)
        self.declare_parameter('xy_tol', 0.025)
        self.declare_parameter('yaw_tol', 0.05)
        self.declare_parameter('no_progress_timeout_sec', 20.0)

        for p in (
            'kp_lin', 'kd_lin', 'kp_ang', 'kd_ang', 'vmax', 'wmax',
            'min_speed', 'min_ang_speed', 'xy_tol', 'yaw_tol',
            'no_progress_timeout_sec',
        ):
            setattr(self, f'_{p}', float(self.get_parameter(p).value))

        self._robot = self.get_parameter('robot').value
        self._odom = None
        self._lock = threading.Lock()
        self._busy = False

        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self._on_odom, 10)
        self._cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self._result_pub = self.create_publisher(String, '/pp/nav/result', 10)
        self.create_subscription(String, '/pp/nav/goto', self._on_goto, 10)
        self.get_logger().info(f'Go-to-goal ready robot={self._robot}')

    def _on_odom(self, msg: Odometry):
        self._odom = msg

    def _stop(self):
        self._cmd_pub.publish(Twist())

    def _publish_result(self, cmd_id: str, ok: bool, message: str):
        msg = String()
        msg.data = json.dumps({'cmd_id': cmd_id, 'success': ok, 'message': message, 'type': 'goto'})
        self._result_pub.publish(msg)

    def _on_goto(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except Exception as exc:
            self._publish_result('', False, f'bad json: {exc}')
            return
        threading.Thread(target=self._run, args=(cmd,), daemon=True).start()

    def _run(self, cmd: dict):
        with self._lock:
            if self._busy:
                self._publish_result(str(cmd.get('cmd_id', '')), False, 'busy')
                return
            self._busy = True
        cmd_id = str(cmd.get('cmd_id', ''))
        gx = float(cmd.get('x', 0.0))
        gy = float(cmd.get('y', 0.0))
        gyaw = float(cmd.get('yaw', 0.0))
        xy_tol = float(cmd.get('xy_tol', self._xy_tol))
        yaw_tol = float(cmd.get('yaw_tol', self._yaw_tol))
        timeout = float(cmd.get('timeout_sec', self._no_progress_timeout_sec))
        ignore_yaw = bool(cmd.get('ignore_yaw', False))

        t0 = time.time()
        last_progress = time.time()
        best = 1e9
        prev_ex = prev_ey = prev_eyaw = 0.0
        ok = False
        message = 'timeout'
        try:
            while rclpy.ok() and time.time() - t0 < timeout:
                if self._odom is None:
                    time.sleep(0.05)
                    continue
                ox = self._odom.pose.pose.position.x
                oy = self._odom.pose.pose.position.y
                oyaw = yaw_of(self._odom)
                dx = gx - ox
                dy = gy - oy
                dist = math.hypot(dx, dy)
                eyaw = normalize(gyaw - oyaw)
                if dist < best - 0.005:
                    best = dist
                    last_progress = time.time()
                if time.time() - last_progress > self._no_progress_timeout_sec:
                    message = 'no progress'
                    break

                # Body frame error
                c, s = math.cos(oyaw), math.sin(oyaw)
                ex = c * dx + s * dy
                ey = -s * dx + c * dy

                if dist <= xy_tol and (ignore_yaw or abs(eyaw) <= yaw_tol):
                    ok, message = True, 'arrived'
                    break

                vx = self._kp_lin * ex + self._kd_lin * (ex - prev_ex)
                vy = self._kp_lin * ey + self._kd_lin * (ey - prev_ey)
                wz = self._kp_ang * eyaw + self._kd_ang * (eyaw - prev_eyaw)
                prev_ex, prev_ey, prev_eyaw = ex, ey, eyaw

                # Clamp
                speed = math.hypot(vx, vy)
                if speed > self._vmax > 0:
                    scale = self._vmax / speed
                    vx *= scale
                    vy *= scale
                wz = max(-self._wmax, min(self._wmax, wz))

                # Min speeds to overcome static friction when far
                if dist > xy_tol * 2 and speed < self._min_speed and speed > 1e-4:
                    scale = self._min_speed / speed
                    vx *= scale
                    vy *= scale
                if not ignore_yaw and abs(eyaw) > yaw_tol and abs(wz) < self._min_ang_speed:
                    wz = math.copysign(self._min_ang_speed, wz)

                # Diff-drive: drop lateral
                if self._robot in ('allterrain', 'grip'):
                    vy = 0.0

                # Prefer rotate-in-place when heading error large and close
                if dist < xy_tol * 3 and abs(eyaw) > 0.4 and not ignore_yaw:
                    vx = 0.0
                    vy = 0.0

                cmd_msg = Twist()
                cmd_msg.linear.x = float(vx)
                cmd_msg.linear.y = float(vy)
                cmd_msg.angular.z = float(wz)
                self._cmd_pub.publish(cmd_msg)
                time.sleep(0.05)
        finally:
            self._stop()
            with self._lock:
                self._busy = False
            self._publish_result(cmd_id, ok, message)


def main(args=None):
    rclpy.init(args=args)
    node = GoToGoalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
