#!/usr/bin/env python3
"""TEMP: log imu_component /imu (+ /gyro_z yaw deg) for EKF jitter analysis.
Delete after the drive test.
"""
from __future__ import annotations

import csv
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

LOG_PATH = Path('/home/vmx/skill23_ws/tmp_imu_ekf_drive.csv')
MARK_PATH = Path('/home/vmx/skill23_ws/tmp_imu_ekf_marks.txt')


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    """Yaw (rad) from quaternion, ZYX / ROS convention."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class ImuEkfLogger(Node):
    def __init__(self) -> None:
        super().__init__('tmp_imu_ekf_logger')
        self._t0 = time.monotonic()
        self._n = 0
        self._gyro_z_deg: float | None = None
        self._fh = LOG_PATH.open('w', newline='')
        self._w = csv.writer(self._fh)
        self._w.writerow([
            't_mono', 'stamp_sec', 'stamp_nsec',
            'yaw_rad', 'yaw_deg', 'gyro_z_topic_deg',
            'wx', 'wy', 'wz',
            'ax', 'ay', 'az',
        ])
        self._fh.flush()
        MARK_PATH.write_text(f'SESSION_START {time.time():.3f}\n')

        self.create_subscription(Imu, '/imu', self._on_imu, 50)
        self.create_subscription(Float32, '/gyro_z', self._on_gyro_z, 50)
        self.get_logger().info(f'Logging /imu -> {LOG_PATH}')

    def _on_gyro_z(self, msg: Float32) -> None:
        self._gyro_z_deg = float(msg.data)

    def _on_imu(self, msg: Imu) -> None:
        q = msg.orientation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        t = time.monotonic() - self._t0
        gz = '' if self._gyro_z_deg is None else f'{self._gyro_z_deg:.6f}'
        self._w.writerow([
            f'{t:.4f}',
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
            f'{yaw:.6f}',
            f'{math.degrees(yaw):.4f}',
            gz,
            f'{msg.angular_velocity.x:.6f}',
            f'{msg.angular_velocity.y:.6f}',
            f'{msg.angular_velocity.z:.6f}',
            f'{msg.linear_acceleration.x:.6f}',
            f'{msg.linear_acceleration.y:.6f}',
            f'{msg.linear_acceleration.z:.6f}',
        ])
        self._n += 1
        if self._n % 20 == 0:
            self._fh.flush()
            self.get_logger().info(
                f'n={self._n} yaw={math.degrees(yaw):+.2f}deg '
                f'w=[{msg.angular_velocity.x:+.3f},{msg.angular_velocity.y:+.3f},{msg.angular_velocity.z:+.3f}] '
                f'a=[{msg.linear_acceleration.x:+.3f},{msg.linear_acceleration.y:+.3f},{msg.linear_acceleration.z:+.3f}]'
            )

    def destroy_node(self) -> bool:
        try:
            self._fh.flush()
            self._fh.close()
        except Exception:
            pass
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = ImuEkfLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
