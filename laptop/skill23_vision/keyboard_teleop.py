#!/usr/bin/env python3
"""Keyboard teleop backup — drive + live view + YOLO / AprilTag capture.

Run on the **laptop** (same ROS_DOMAIN_ID as Pi). Publishes /cmd_vel directly.

IMPORTANT: do NOT also run joy_node / gamepad at the same time — both would
fight over /cmd_vel. For keyboard-only:
  - skip ./start_teleop_capture.sh (that starts joy_node)
  - run this instead

Keys (focus the OpenCV window):
  W / S     forward / back
  A / D     strafe left / right   (mecanum / xdrive)
  Q / E     rotate left / right
  SPACE     stop
  Y         YOLO capture (same save as teleop_capture)
  T         AprilTag capture
  H         print help
  ESC / X   quit

Speed: --lin 0.2 --ang 0.2  (same scale idea as teleop YAML)
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Pose2D, Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float64MultiArray

# Reuse detectors / image helpers from teleop_capture (no network)
from teleop_capture import (
    _color_to_bgr,
    _compressed_to_bgr,
    _yolo_path,
    apriltag_backend_info,
    run_apriltag,
    run_yolo,
)


HELP = """
  W/S  forward/back     A/D  strafe      Q/E  rotate
  SPACE stop            Y  YOLO          T  AprilTag
  H help                ESC/X quit
"""


class KeyboardTeleop(Node):
    def __init__(
            self,
            *,
            out_dir: Path,
            lin: float,
            ang: float,
            conf: float) -> None:
        super().__init__('keyboard_teleop')
        self._out = out_dir
        self._out.mkdir(parents=True, exist_ok=True)
        self._txt = self._out / 'detections.txt'
        self._lin = lin
        self._ang = ang
        self._conf = conf

        self._frame: Optional[np.ndarray] = None
        self._pose = (0.0, 0.0, 0.0)
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0
        self._busy = False
        self._quit = False

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            CompressedImage, '/camera/color/image_raw/compressed',
            self._on_compressed, qos_profile_sensor_data)
        self.create_subscription(
            Image, '/camera/color/image_raw',
            self._on_image, qos_profile_sensor_data)
        self.create_subscription(Pose2D, '/robot_pose', self._on_pose, 10)
        self.create_subscription(
            Float64MultiArray, '/robot_pose_xyyaw', self._on_xyyaw, 10)

        # Publish cmd_vel at 20 Hz (zeros when no key held)
        self.create_timer(0.05, self._pub_cmd)

        self.get_logger().info(
            f'keyboard_teleop ready  lin={lin} ang={ang}  out={self._out}')
        self.get_logger().info(f'AprilTag backend: {apriltag_backend_info()}')
        self.get_logger().info(f'YOLO weights: {_yolo_path()}')
        self.get_logger().info(HELP.replace('\n', ' | '))

    def _on_compressed(self, msg: CompressedImage) -> None:
        frame = _compressed_to_bgr(msg)
        if frame is not None:
            self._frame = frame

    def _on_image(self, msg: Image) -> None:
        if self._frame is not None:
            return
        frame = _color_to_bgr(msg)
        if frame is not None:
            self._frame = frame

    def _on_pose(self, msg: Pose2D) -> None:
        self._pose = (float(msg.x), float(msg.y), float(msg.theta))

    def _on_xyyaw(self, msg: Float64MultiArray) -> None:
        if len(msg.data) >= 3:
            self._pose = (float(msg.data[0]), float(msg.data[1]), float(msg.data[2]))

    def _pub_cmd(self) -> None:
        tw = Twist()
        tw.linear.x = self._vx
        tw.linear.y = self._vy
        tw.angular.z = self._wz
        self._cmd_pub.publish(tw)

    def stop(self) -> None:
        self._vx = self._vy = self._wz = 0.0
        self._pub_cmd()

    def handle_key(self, key: int) -> None:
        if key < 0:
            # no new key event — keep last velocity (OpenCV has no key-up)
            return

        k = key & 0xFF

        if k in (27, ord('x'), ord('X')):  # ESC / X
            self._quit = True
            self.stop()
            return
        if k in (ord('h'), ord('H')):
            print(HELP)
            return
        if k == ord(' '):
            self.stop()
            return

        # motion — latches until SPACE / other motion key
        if k in (ord('w'), ord('W')):
            self._vx, self._vy, self._wz = self._lin, 0.0, 0.0
        elif k in (ord('s'), ord('S')):
            self._vx, self._vy, self._wz = -self._lin, 0.0, 0.0
        elif k in (ord('a'), ord('A')):
            self._vx, self._vy, self._wz = 0.0, self._lin, 0.0
        elif k in (ord('d'), ord('D')):
            self._vx, self._vy, self._wz = 0.0, -self._lin, 0.0
        elif k in (ord('q'), ord('Q')):
            self._vx, self._vy, self._wz = 0.0, 0.0, self._ang
        elif k in (ord('e'), ord('E')):
            self._vx, self._vy, self._wz = 0.0, 0.0, -self._ang
        elif k in (ord('y'), ord('Y')):
            if not self._busy:
                self._busy = True
                try:
                    self._capture_yolo()
                finally:
                    self._busy = False
        elif k in (ord('t'), ord('T')):
            if not self._busy:
                self._busy = True
                try:
                    self._capture_apriltag()
                finally:
                    self._busy = False

    def _stamp(self) -> str:
        from datetime import datetime
        return datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]

    def _pose_str(self) -> str:
        x, y, yaw = self._pose
        return f'x={x:.3f} y={y:.3f} yaw={yaw:.3f}'

    def _append_txt(self, line: str) -> None:
        with open(self._txt, 'a', encoding='utf-8') as f:
            f.write(line.rstrip() + '\n')

    def _capture_yolo(self) -> None:
        frame = None if self._frame is None else self._frame.copy()
        if frame is None:
            self.get_logger().warn('YOLO: no camera frame yet')
            return
        ts = self._stamp()
        pose = self._pose_str()
        result = run_yolo(frame, conf=self._conf)
        vis = frame.copy()
        lines = [f'[YOLO] {ts} robot_pose {pose}']
        if result['ok']:
            for h in result['hits']:
                x1, y1, x2, y2 = [int(v) for v in h['box']]
                cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 200, 0), 2)
                cv2.putText(
                    vis, f"{h['class']} {h['conf']:.2f}",
                    (x1, max(20, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 0), 2)
                lines.append(
                    f"  object={h['class']} conf={h['conf']} "
                    f"img_xy=({h['cx']:.1f},{h['cy']:.1f}) "
                    f"robot_pose=({self._pose[0]:.3f},{self._pose[1]:.3f},"
                    f"{self._pose[2]:.3f})")
        else:
            lines.append(f"  {result['message']}")
        cv2.putText(
            vis, f'YOLO  {pose}  {ts}', (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        path = self._out / f'yolo_{ts}.jpg'
        cv2.imwrite(str(path), vis)
        for line in lines:
            self._append_txt(line)
        self._append_txt(f'  image={path.name}')
        self.get_logger().info(f'YOLO saved {path.name} — {result["message"]}')
        cv2.imshow('YOLO capture', vis)

    def _capture_apriltag(self) -> None:
        frame = None if self._frame is None else self._frame.copy()
        if frame is None:
            self.get_logger().warn('AprilTag: no camera frame yet')
            return
        ts = self._stamp()
        pose = self._pose_str()
        result = run_apriltag(frame)
        vis = frame.copy()
        lines = [f'[APRILTAG] {ts} robot_pose {pose}']
        if result['ok']:
            for h in result['hits']:
                pts = np.array(h['corners'], dtype=np.int32)
                cv2.polylines(vis, [pts], True, (255, 128, 0), 2)
                cv2.putText(
                    vis, f"id={h['id']}", (int(h['cx']), int(h['cy']) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 128, 0), 2)
                lines.append(
                    f"  id={h['id']} img_xy=({h['cx']:.1f},{h['cy']:.1f}) "
                    f"robot_pose=({self._pose[0]:.3f},{self._pose[1]:.3f},"
                    f"{self._pose[2]:.3f})")
        else:
            lines.append(f"  {result['message']}")
        cv2.putText(
            vis, f'AprilTag  {pose}  {ts}', (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        path = self._out / f'apriltag_{ts}.jpg'
        cv2.imwrite(str(path), vis)
        for line in lines:
            self._append_txt(line)
        self._append_txt(f'  image={path.name}')
        self.get_logger().info(
            f'AprilTag saved {path.name} — {result["message"]}')
        cv2.imshow('AprilTag capture', vis)

    def draw_live(self) -> None:
        if self._frame is None:
            blank = np.zeros((240, 320, 3), dtype=np.uint8)
            cv2.putText(
                blank, 'waiting for /camera/...', (20, 120),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            cv2.imshow('keyboard teleop', blank)
            return
        vis = self._frame.copy()
        x, y, yaw = self._pose
        cv2.putText(
            vis,
            f'LIVE  x={x:.2f} y={y:.2f} yaw={yaw:.2f}  '
            f'cmd vx={self._vx:.2f} vy={self._vy:.2f} wz={self._wz:.2f}',
            (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        cv2.putText(
            vis, 'W/S move  A/D strafe  Q/E turn  Y yolo  T tag  SPACE stop',
            (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 255, 200), 1)
        cv2.imshow('keyboard teleop', vis)


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description='Keyboard teleop + capture')
    parser.add_argument(
        '--out', default=str(Path(__file__).resolve().parent / 'captures'))
    parser.add_argument('--lin', type=float, default=0.2, help='linear scale')
    parser.add_argument('--ang', type=float, default=0.2, help='angular scale')
    parser.add_argument('--conf', type=float, default=0.4)
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = KeyboardTeleop(
        out_dir=Path(args.out).expanduser(),
        lin=args.lin,
        ang=args.ang,
        conf=args.conf,
    )

    # Spin ROS in background; OpenCV waitKey needs main thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print(HELP)
    try:
        while rclpy.ok() and not node._quit:
            node.draw_live()
            key = cv2.waitKey(50)
            node.handle_key(key)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
