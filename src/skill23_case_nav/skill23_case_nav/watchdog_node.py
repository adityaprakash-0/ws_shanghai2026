"""Watchdog node — port of src/watchdog_node.cpp."""

from __future__ import annotations

import json
import math
from collections import deque
from typing import Deque, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import String


class WatchdogNode(Node):
    def __init__(self) -> None:
        super().__init__("cn_watchdog")

        self.declare_parameter("us_left_topic", "/ultrasonic_left/range")
        self.declare_parameter("us_right_topic", "/ultrasonic_right/range")
        self.declare_parameter("front_scan_topic", "/scan/front")
        self.declare_parameter("wall_detect_m", 0.45)
        self.declare_parameter("open_thresh_m", 0.55)
        self.declare_parameter("front_block_m", 0.20)
        self.declare_parameter("lidar_fov_deg", 90.0)
        self.declare_parameter("debounce_count", 3)
        self.declare_parameter("open_debounce_count", 2)

        self.wall_detect_m_ = float(self.get_parameter("wall_detect_m").value)
        self.open_thresh_m_ = float(self.get_parameter("open_thresh_m").value)
        self.front_block_m_ = float(self.get_parameter("front_block_m").value)
        self.lidar_half_fov_ = (
            float(self.get_parameter("lidar_fov_deg").value) * math.pi / 360.0
        )
        self.debounce_count_ = max(
            1, int(self.get_parameter("debounce_count").value)
        )
        self.open_debounce_ = max(
            1, int(self.get_parameter("open_debounce_count").value)
        )

        self.us_l_: Optional[float] = None
        self.us_r_: Optional[float] = None
        self.front_range_: Optional[float] = None
        self.hist_wall_l_: Deque[bool] = deque()
        self.hist_wall_r_: Deque[bool] = deque()
        self.hist_open_l_: Deque[bool] = deque()
        self.hist_open_r_: Deque[bool] = deque()
        self.hist_front_: Deque[bool] = deque()

        self.create_subscription(
            Range,
            str(self.get_parameter("us_left_topic").value),
            lambda msg: self._set_us("l", msg),
            10,
        )
        self.create_subscription(
            Range,
            str(self.get_parameter("us_right_topic").value),
            lambda msg: self._set_us("r", msg),
            10,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("front_scan_topic").value),
            self._on_scan,
            qos_profile_sensor_data,
        )

        self.pub_ = self.create_publisher(String, "/case_nav/watchdog", 10)
        self.create_timer(0.05, self._tick)
        self.get_logger().info(
            "Watchdog: US L/R + front lidar ±%.0f° (no IR)"
            % (float(self.get_parameter("lidar_fov_deg").value) * 0.5)
        )

    def _set_us(self, side: str, msg: Range) -> None:
        r = float(msg.range)
        val: Optional[float] = None
        if math.isfinite(r) and r > 0.0:
            val = r
        if side == "l":
            self.us_l_ = val
        else:
            self.us_r_ = val

    def _on_scan(self, msg: LaserScan) -> None:
        # Min range in FOV (±lidar_fov/2). Accept close hits even if driver
        # range_min is higher (otherwise <7 cm returns are dropped).
        hard_min = 0.02
        rmin = min(float(msg.range_min), hard_min) if msg.range_min > 0 else hard_min
        best: Optional[float] = None
        angle = float(msg.angle_min)
        for r in msg.ranges:
            rr = float(r)
            if (
                abs(angle) <= self.lidar_half_fov_
                and math.isfinite(rr)
                and rr >= rmin
                and rr < msg.range_max
            ):
                if best is None or rr < best:
                    best = rr
            angle += float(msg.angle_increment)
        self.front_range_ = best

    @staticmethod
    def _push_hist(hist: Deque[bool], val: bool, maxlen: int) -> None:
        hist.append(val)
        while len(hist) > maxlen:
            hist.popleft()

    @staticmethod
    def _stable(hist: Deque[bool], maxlen: int) -> bool:
        if len(hist) < maxlen:
            return False
        tail = list(hist)[-maxlen:]
        return all(tail)

    def _tick(self) -> None:
        wall_l_raw = self.us_l_ is not None and self.us_l_ < self.wall_detect_m_
        wall_r_raw = self.us_r_ is not None and self.us_r_ < self.wall_detect_m_
        open_l_raw = self.us_l_ is None or self.us_l_ > self.open_thresh_m_
        open_r_raw = self.us_r_ is None or self.us_r_ > self.open_thresh_m_
        front_raw = (
            self.front_range_ is not None
            and self.front_range_ < self.front_block_m_
        )

        self._push_hist(self.hist_wall_l_, wall_l_raw, self.debounce_count_)
        self._push_hist(self.hist_wall_r_, wall_r_raw, self.debounce_count_)
        self._push_hist(self.hist_open_l_, open_l_raw, self.open_debounce_)
        self._push_hist(self.hist_open_r_, open_r_raw, self.open_debounce_)
        self._push_hist(self.hist_front_, front_raw, self.debounce_count_)

        wall_l = self._stable(self.hist_wall_l_, self.debounce_count_)
        wall_r = self._stable(self.hist_wall_r_, self.debounce_count_)
        open_l = self._stable(self.hist_open_l_, self.open_debounce_)
        open_r = self._stable(self.hist_open_r_, self.open_debounce_)
        # Instant OR stable — don't hide close obstacles during debounce fill
        front_blocked = front_raw or self._stable(
            self.hist_front_, self.debounce_count_
        )

        payload = {
            "wall_left": wall_l,
            "wall_right": wall_r,
            "open_left": open_l,
            "open_right": open_r,
            "front_blocked": front_blocked,
            "front_blocked_raw": front_raw,
            "front_block_thresh_m": round(self.front_block_m_, 3),
            "us_left_m": (
                round(self.us_l_ * 1000.0) / 1000.0
                if self.us_l_ is not None
                else None
            ),
            "us_right_m": (
                round(self.us_r_ * 1000.0) / 1000.0
                if self.us_r_ is not None
                else None
            ),
            "front_range_m": (
                round(self.front_range_ * 1000.0) / 1000.0
                if self.front_range_ is not None
                else None
            ),
        }

        msg = String()
        msg.data = json.dumps(payload)
        self.pub_.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WatchdogNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
