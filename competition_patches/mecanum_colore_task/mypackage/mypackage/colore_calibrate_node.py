#!/usr/bin/env python3
"""Collect Colore samples and print a REFERENCE_XY entry to copy."""

from __future__ import annotations

import statistics
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from studica_control.msg import ColoreColorMsg


def xyz_to_xy(x_value: float, y_value: float, z_value: float) -> Tuple[float, float]:
    total = x_value + y_value + z_value
    if total <= 1e-9:
        raise ValueError('XYZ total is zero')
    return x_value / total, y_value / total


class ColoreCalibrateNode(Node):
    def __init__(self) -> None:
        super().__init__('colore_calibrate')
        self.declare_parameter('color_name', '')
        self.declare_parameter('sample_count', 40)
        self.declare_parameter('color_info_topic', '/colore0/color_info')
        self.declare_parameter('min_brightness', 0.001)
        self._name = str(self.get_parameter('color_name').value).strip().lower()
        self._sample_count = int(self.get_parameter('sample_count').value)
        self._min_brightness = float(self.get_parameter('min_brightness').value)
        topic = str(self.get_parameter('color_info_topic').value)
        if not self._name:
            raise ValueError('color_name is empty; use colore_calibrate.launch.py')
        if self._sample_count < 5:
            raise ValueError('sample_count must be at least 5')
        self._samples: List[Tuple[float, float]] = []
        self._finished = False
        self.create_subscription(ColoreColorMsg, topic, self._on_color, 10)
        self.create_timer(2.0, self._waiting_status)
        print(
            f'\nCollecting {self._sample_count} readings for "{self._name}" '
            f'from {topic}. Keep the target still...\n', flush=True)

    def _waiting_status(self) -> None:
        if not self._finished and not self._samples:
            self.get_logger().warn(
                'No valid readings yet — check Colore USB and /colore0/color_info')

    def _on_color(self, msg: ColoreColorMsg) -> None:
        if self._finished or float(msg.y) < self._min_brightness:
            return
        try:
            xy = xyz_to_xy(float(msg.x), float(msg.y), float(msg.z))
        except ValueError:
            return
        self._samples.append(xy)
        count = len(self._samples)
        if count == 1 or count % 10 == 0:
            print(
                f'  reading {count}/{self._sample_count}: '
                f'xy=({xy[0]:.4f}, {xy[1]:.4f})', flush=True)
        if count >= self._sample_count:
            self._print_result()

    def _print_result(self) -> None:
        self._finished = True
        xs = [sample[0] for sample in self._samples]
        ys = [sample[1] for sample in self._samples]
        ref_x = statistics.median(xs)
        ref_y = statistics.median(ys)
        spread_x = statistics.pstdev(xs)
        spread_y = statistics.pstdev(ys)
        print('\n========== CALIBRATION COMPLETE ==========', flush=True)
        print(f'Color: {self._name}', flush=True)
        print(f'Samples: {len(self._samples)}', flush=True)
        print(f'Reference xy (median): ({ref_x:.4f}, {ref_y:.4f})', flush=True)
        print(
            f'Reading spread: x={spread_x:.4f}, y={spread_y:.4f} '
            '(smaller is better)', flush=True)
        print('\nCOPY THIS LINE into REFERENCE_XY:', flush=True)
        print(f"    '{self._name}': ({ref_x:.4f}, {ref_y:.4f}),", flush=True)
        print(
            '\nThen rebuild: colcon build --packages-select mypackage '
            '&& source install/setup.bash', flush=True)
        print('==========================================\n', flush=True)
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ColoreCalibrateNode()
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
