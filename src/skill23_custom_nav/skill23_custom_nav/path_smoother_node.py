#!/usr/bin/env python3
"""Path smoother + style mux for skill23_custom_nav.

Subscribes planner output on /custom_nav/planner_path, publishes:
  /custom_nav/smoothed_path  — always the outward-smoothed path
  /custom_nav/path           — selected style (normal | smoothed)

Style from param default or /custom_nav/path_style (std_msgs/String).
"""

from __future__ import annotations

from typing import List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from skill23_custom_nav.node_status import NodeStatusPub
from skill23_custom_nav.planning import decode_edt_cells
from skill23_custom_nav.smoothing import smooth_path_poses
from skill23_custom_nav.utils import quat_from_yaw, yaw_from_quat


class PathSmootherNode(Node):
    def __init__(self) -> None:
        super().__init__('cn_path_smoother')
        self.declare_parameter('path_style', 'smoothed')  # normal | smoothed
        self.declare_parameter('occupied_thresh', 50)
        self.declare_parameter('min_turn_rad', 0.25)
        self.declare_parameter('fillet_radius_m', 0.18)
        self.declare_parameter('sample_step_m', 0.04)
        self.declare_parameter('min_clearance_m', 0.14)
        self.declare_parameter('max_offset_m', 0.08)
        self.declare_parameter('map_frame', 'map')

        style = str(self.get_parameter('path_style').value).strip().lower()
        self.path_style = 'smoothed' if style not in ('normal', 'raw', 'planner') else 'normal'
        if style in ('smoothed', 'smooth'):
            self.path_style = 'smoothed'
        self.occupied_thresh = int(self.get_parameter('occupied_thresh').value)
        self.min_turn = float(self.get_parameter('min_turn_rad').value)
        self.fillet_r = float(self.get_parameter('fillet_radius_m').value)
        self.sample_step = float(self.get_parameter('sample_step_m').value)
        self.min_clear = float(self.get_parameter('min_clearance_m').value)
        self.max_offset = float(self.get_parameter('max_offset_m').value)
        self.map_frame = str(self.get_parameter('map_frame').value)

        self.map_msg: Optional[OccupancyGrid] = None
        self.occupancy: Optional[np.ndarray] = None
        self.static_edt: Optional[np.ndarray] = None
        self.planner_path: Optional[Path] = None
        self.smoothed_path: Optional[Path] = None

        self.status = NodeStatusPub(self, 'path_smoother')
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.active_pub = self.create_publisher(Path, '/custom_nav/path', 10)
        self.smoothed_pub = self.create_publisher(Path, '/custom_nav/smoothed_path', 10)

        self.create_subscription(Path, '/custom_nav/planner_path', self._on_planner_path, 10)
        self.create_subscription(String, '/custom_nav/path_style', self._on_style, 10)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, latched)
        self.create_subscription(
            OccupancyGrid, '/custom_nav/static_edt', self._on_static_edt, latched)

        self.status.set('idle', f'style={self.path_style} — waiting for planner_path')

    def _on_style(self, msg: String) -> None:
        raw = msg.data.strip().lower()
        if raw in ('normal', 'raw', 'planner'):
            style = 'normal'
        elif raw in ('smoothed', 'smooth'):
            style = 'smoothed'
        else:
            return
        if style == self.path_style:
            return
        self.path_style = style
        self.get_logger().info(f'path_style → {self.path_style}')
        self._republish_active()
        self.status.set('idle', f'style={self.path_style}')

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg
        self.occupancy = np.array(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width)

    def _on_static_edt(self, msg: OccupancyGrid) -> None:
        if msg.info.width == 0:
            return
        occ = self.occupancy
        if occ is None or occ.shape != (msg.info.height, msg.info.width):
            if self.map_msg is not None:
                occ = np.array(self.map_msg.data, dtype=np.int16).reshape(
                    self.map_msg.info.height, self.map_msg.info.width)
                self.occupancy = occ
        self.static_edt = decode_edt_cells(
            msg.data, msg.info.height, msg.info.width, msg.info.resolution,
            self.occupied_thresh, occ)

    def _poses_from_path(self, path: Path) -> List[tuple]:
        poses = []
        for ps in path.poses:
            q = ps.pose.orientation
            yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
            poses.append((
                float(ps.pose.position.x),
                float(ps.pose.position.y),
                yaw,
            ))
        return poses

    def _path_from_poses(self, poses: List[tuple], stamp) -> Path:
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = self.map_frame
        for x, y, yaw in poses:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            qx, qy, qz, qw = quat_from_yaw(float(yaw))
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            path.poses.append(ps)
        return path

    def _smooth(self, planner: Path) -> Path:
        poses = self._poses_from_path(planner)
        if self.static_edt is None or self.map_msg is None or len(poses) < 3:
            return planner
        info = self.map_msg.info
        smoothed = smooth_path_poses(
            poses,
            self.static_edt,
            info.origin.position.x,
            info.origin.position.y,
            float(info.resolution),
            occupancy=self.occupancy,
            occupied_thresh=self.occupied_thresh,
            min_turn_rad=self.min_turn,
            fillet_radius_m=self.fillet_r,
            sample_step_m=self.sample_step,
            min_clearance_m=self.min_clear,
            max_offset_m=self.max_offset,
        )
        return self._path_from_poses(smoothed, planner.header.stamp)

    def _on_planner_path(self, msg: Path) -> None:
        if not msg.poses:
            self.planner_path = msg
            self.smoothed_path = msg
            self.smoothed_pub.publish(msg)
            self.active_pub.publish(msg)
            self.status.set('idle', 'empty planner path')
            return
        self.planner_path = msg
        self.smoothed_path = self._smooth(msg)
        self.smoothed_pub.publish(self.smoothed_path)
        self._republish_active()
        n_raw = len(msg.poses)
        n_sm = len(self.smoothed_path.poses)
        self.status.set(
            'done',
            f'{n_raw}→{n_sm} pts style={self.path_style}')
        self.get_logger().info(
            f'smoothed planner {n_raw} → {n_sm} pts (active={self.path_style})')

    def _republish_active(self) -> None:
        if self.path_style == 'normal':
            path = self.planner_path
        else:
            path = self.smoothed_path if self.smoothed_path is not None else self.planner_path
        if path is None:
            return
        self.active_pub.publish(path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathSmootherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
