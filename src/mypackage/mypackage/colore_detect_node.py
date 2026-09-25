#!/usr/bin/env python3
"""Watch Studica Colore for a named color (red/green/blue/orange/purple).

Subscribes: /colore0/color_info  (CIE XYZ + RGB)
            /colore_detect/watch (std_msgs/String) — color name to watch, "" = off

Publishes:  /colore_detect/found (Bool) — True once when match seen while watching
            /colore_detect/label (String) — last matched label

Matching uses chromaticity xy = (X/(X+Y+Z), Y/(X+Y+Z)) vs REFERENCE_XY.
Tune REFERENCES or match_threshold if lighting changes (see README).
"""

from __future__ import annotations

import math
import time
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

try:
    from studica_control.msg import ColoreColorMsg
except ImportError:  # pragma: no cover
    ColoreColorMsg = None


# Default CIE xy targets (approximate — calibrate on your arena lighting)
REFERENCE_XY: Dict[str, Tuple[float, float]] = {
    'red': (0.64, 0.33),
    'green': (0.30, 0.60),
    'blue': (0.15, 0.20),
    'orange': (0.55, 0.41),
    'purple': (0.28, 0.18),
}

ALLOWED = tuple(REFERENCE_XY.keys())


class ColoreDetectNode(Node):
    def __init__(self) -> None:
        super().__init__('colore_detect')
        self.declare_parameter('color_info_topic', '/colore0/color_info')
        self.declare_parameter('match_threshold', 0.08)
        self.declare_parameter('min_brightness', 0.02)  # reject near-black XYZ
        # When watching, print live xy this often (Hz). 0 = off. Use while calibrating.
        self.declare_parameter('log_xy_hz', 2.0)

        self._thresh = float(self.get_parameter('match_threshold').value)
        self._min_y = float(self.get_parameter('min_brightness').value)
        self._log_xy_hz = float(self.get_parameter('log_xy_hz').value)
        info_topic = str(self.get_parameter('color_info_topic').value)

        self._watch: Optional[str] = None
        self._found_latched = False
        self._last_xy_log = 0.0

        self._found_pub = self.create_publisher(Bool, '/colore_detect/found', 10)
        self._label_pub = self.create_publisher(String, '/colore_detect/label', 10)
        self.create_subscription(String, '/colore_detect/watch', self._on_watch, 10)

        if ColoreColorMsg is None:
            self.get_logger().error('studica_control.msg.ColoreColorMsg missing — rebuild studica_control')
        else:
            self.create_subscription(ColoreColorMsg, info_topic, self._on_info, 10)

        self.get_logger().info(
            f'colore_detect ready  info={info_topic}  colors={list(ALLOWED)}  '
            f'thresh={self._thresh}  log_xy_hz={self._log_xy_hz}')
        print(
            f'colore_detect ready — colors={list(ALLOWED)} thresh={self._thresh}',
            flush=True)

    def _on_watch(self, msg: String) -> None:
        name = (msg.data or '').strip().lower()
        if not name:
            self._watch = None
            self._found_latched = False
            self.get_logger().info('colore_detect WATCH off')
            print('colore_detect WATCH off', flush=True)
            return
        if name not in REFERENCE_XY:
            self.get_logger().warn(f'unknown color "{name}" — allowed: {ALLOWED}')
            print(f'unknown color "{name}" — allowed: {ALLOWED}', flush=True)
            return
        self._watch = name
        self._found_latched = False
        ref = REFERENCE_XY[name]
        self.get_logger().info(
            f'colore_detect WATCH → {name}  ref_xy=({ref[0]:.3f},{ref[1]:.3f})  '
            f'thresh={self._thresh}')
        print(
            f'colore_detect WATCH → {name}  ref=({ref[0]:.3f},{ref[1]:.3f})',
            flush=True)

    def _on_info(self, msg) -> None:
        if self._watch is None or self._found_latched:
            return
        X, Y, Z = float(msg.x), float(msg.y), float(msg.z)
        s = X + Y + Z
        if s < 1e-6 or Y < self._min_y:
            return
        x, y = X / s, Y / s
        ref = REFERENCE_XY[self._watch]
        dist = math.hypot(x - ref[0], y - ref[1])

        # Live calibration log (does not affect match speed)
        if self._log_xy_hz > 0.0:
            now = time.monotonic()
            if now - self._last_xy_log >= 1.0 / self._log_xy_hz:
                self._last_xy_log = now
                line = (
                    f'  xy=({x:.3f},{y:.3f})  dist_to_{self._watch}={dist:.3f}  '
                    f'thresh={self._thresh}  RGB=({msg.r},{msg.g},{msg.b})')
                self.get_logger().info(line)
                print(line, flush=True)

        if dist > self._thresh:
            return
        self._found_latched = True
        lab = String()
        lab.data = self._watch
        self._label_pub.publish(lab)
        found = Bool()
        found.data = True
        self._found_pub.publish(found)
        self.get_logger().info(
            f'COLOR FOUND {self._watch}  xy=({x:.3f},{y:.3f}) dist={dist:.3f}')
        print(
            f'>>> COLOR FOUND {self._watch}  xy=({x:.3f},{y:.3f}) dist={dist:.3f}',
            flush=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ColoreDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
