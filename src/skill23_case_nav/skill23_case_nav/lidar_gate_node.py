"""Lidar gate node — port of src/lidar_gate_node.cpp."""

from __future__ import annotations

import json
import math
from typing import Any, List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class LidarGateNode(Node):
    def __init__(self) -> None:
        super().__init__("cn_lidar_gate")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("lidar_stamp_max_range", 2.0)
        self.declare_parameter("lidar_idle_sleep_sec", 3.0)

        self.max_range_ = float(self.get_parameter("lidar_stamp_max_range").value)
        self.idle_sleep_sec_ = float(
            self.get_parameter("lidar_idle_sleep_sec").value
        )

        self.awake_ = False
        self.last_stamp_time_ = 0.0
        self.last_scan_time_ = 0.0
        self.pose_: Optional[PoseStamped] = None

        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(PoseStamped, "/case_nav/pose", self._on_pose, 10)
        self.create_subscription(String, "/case_nav/lidar_wake", self._on_wake, 10)

        self.patch_pub_ = self.create_publisher(String, "/case_nav/map/patch", 10)
        self.status_pub_ = self.create_publisher(String, "/case_nav/lidar_status", 10)
        self.create_timer(0.2, self._tick)
        self.get_logger().info("Lidar gate: sleeping until /case_nav/lidar_wake")

    def _on_pose(self, msg: PoseStamped) -> None:
        self.pose_ = msg

    def _on_wake(self, msg: String) -> None:
        try:
            d: Any = json.loads(msg.data)
        except Exception:
            d = {"wake": True}
        if not d.get("wake", True):
            self.awake_ = False
            return
        self.awake_ = True
        self.last_stamp_time_ = self._now_seconds()
        self.get_logger().info("Lidar WAKE")

    def _on_scan(self, msg: LaserScan) -> None:
        self.last_scan_time_ = self._now_seconds()
        if not self.awake_:
            return
        self.last_stamp_time_ = self.last_scan_time_
        self._stamp_scan(msg)

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self) -> None:
        t = self._now_seconds()
        if (
            self.awake_
            and self.last_stamp_time_ > 0.0
            and (t - self.last_stamp_time_) >= self.idle_sleep_sec_
        ):
            self.awake_ = False
            self.get_logger().info(
                "Lidar SLEEP (no stamps for %.1fs)" % self.idle_sleep_sec_
            )

        st = String()
        st.data = json.dumps({"awake": self.awake_})
        self.status_pub_.publish(st)

    def _stamp_scan(self, scan: LaserScan) -> None:
        if self.pose_ is None:
            return
        px = float(self.pose_.pose.position.x)
        py = float(self.pose_.pose.position.y)
        z = float(self.pose_.pose.orientation.z)
        w = float(self.pose_.pose.orientation.w)
        yaw = math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)

        points: List[dict] = []
        angle = float(scan.angle_min)
        for r in scan.ranges:
            rr = float(r)
            if (
                math.isfinite(rr)
                and rr > scan.range_min
                and rr < min(float(scan.range_max), self.max_range_)
            ):
                wx = px + rr * math.cos(yaw + angle)
                wy = py + rr * math.sin(yaw + angle)
                points.append({"x": wx, "y": wy})
            angle += float(scan.angle_increment)

        if not points:
            return

        step = max(1, len(points) // 40)
        sampled = [points[i] for i in range(0, len(points), step)]
        msg = String()
        msg.data = json.dumps({"points": sampled, "value": 100})
        self.patch_pub_.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarGateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
