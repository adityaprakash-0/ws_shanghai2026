#!/usr/bin/env python3
"""ROS shell for on-demand planning — algorithm lives in planning.py.

Triggers:
  /custom_nav/plan_request   → plan on static EDT
  /custom_nav/replan         → plan on static ∩ temporary local EDT

Output: /custom_nav/path
"""

from __future__ import annotations

from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty

from skill23_custom_nav.node_status import NodeStatusPub
from skill23_custom_nav.planning import LocalEdtInput, decode_edt_cells, plan_path
from skill23_custom_nav.utils import pose_cov_to_stamped, quat_from_yaw, yaw_from_quat


class PlannerNode(Node):
    def __init__(self) -> None:
        super().__init__('cn_planner')
        self.declare_parameter('occupied_thresh', 50)
        self.declare_parameter('wall_cost_weight', 4.0)
        self.declare_parameter('min_clearance_m', 0.08)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('los_sample_step_m', 0.05)

        self.occupied_thresh = int(self.get_parameter('occupied_thresh').value)
        self.clearance_weight = float(self.get_parameter('wall_cost_weight').value)
        self.min_clearance_m = float(self.get_parameter('min_clearance_m').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.los_step = float(self.get_parameter('los_sample_step_m').value)

        self.map_msg: Optional[OccupancyGrid] = None
        self.occupancy: Optional[np.ndarray] = None
        self.static_edt_msg: Optional[OccupancyGrid] = None
        self.static_edt: Optional[np.ndarray] = None
        self.pose: Optional[PoseStamped] = None
        self.pending_goal: Optional[PoseStamped] = None
        self.local_edt_msg: Optional[OccupancyGrid] = None
        self.local_edt_active = False
        self.use_local = False

        self.status = NodeStatusPub(self, 'planner')

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.path_pub = self.create_publisher(Path, '/custom_nav/path', 10)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, latched)
        self.create_subscription(
            OccupancyGrid, '/custom_nav/static_edt', self._on_static_edt, latched)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_pose, 10)
        self.create_subscription(
            OccupancyGrid, '/custom_nav/local_edt', self._on_local_edt, 10)
        self.create_subscription(
            Bool, '/custom_nav/local_edt/active', self._on_local_active, 10)
        self.create_subscription(
            PoseStamped, '/custom_nav/plan_request', self._on_plan_req, 10)
        self.create_subscription(Empty, '/custom_nav/replan', self._on_replan, 10)
        self.status.set('idle', 'waiting for plan request')

    def _refresh_static_edt(self) -> None:
        if self.static_edt_msg is None:
            return
        m = self.static_edt_msg
        self.static_edt = decode_edt_cells(
            m.data, m.info.height, m.info.width, m.info.resolution,
            self.occupied_thresh, self.occupancy)

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg
        self.occupancy = np.array(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width)
        self._refresh_static_edt()
        self.status.set('idle', f'map {msg.info.width}x{msg.info.height}')

    def _on_static_edt(self, msg: OccupancyGrid) -> None:
        self.static_edt_msg = msg
        self._refresh_static_edt()

    def _on_local_edt(self, msg: OccupancyGrid) -> None:
        if msg.info.width == 0 or msg.info.height == 0:
            self.local_edt_msg = None
            return
        self.local_edt_msg = msg

    def _on_local_active(self, msg: Bool) -> None:
        self.local_edt_active = bool(msg.data)
        if not self.local_edt_active:
            self.local_edt_msg = None

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        ps = pose_cov_to_stamped(msg)
        if abs(ps.pose.position.x) > 50.0 or abs(ps.pose.position.y) > 50.0:
            return
        self.pose = ps

    def _on_plan_req(self, msg: PoseStamped) -> None:
        self.pending_goal = msg
        self.use_local = False
        self.status.set(
            'planning',
            f'goal ({msg.pose.position.x:.2f},{msg.pose.position.y:.2f})')
        self._plan_once()

    def _on_replan(self, _msg: Empty) -> None:
        if self.pending_goal is None:
            self.status.set('failed', 'replan with no goal')
            return
        self.use_local = True
        self.status.set('planning', 'replan with static+local EDT')
        self._plan_once()

    def _local_input(self) -> Optional[LocalEdtInput]:
        if not self.use_local or not self.local_edt_active:
            return None
        if self.local_edt_msg is None or self.pose is None:
            return None
        lg = self.local_edt_msg
        q = self.pose.pose.orientation
        return LocalEdtInput(
            data=np.array(lg.data, dtype=np.int16).reshape(
                lg.info.height, lg.info.width),
            origin_x=lg.info.origin.position.x,
            origin_y=lg.info.origin.position.y,
            resolution=float(lg.info.resolution),
            robot_x=self.pose.pose.position.x,
            robot_y=self.pose.pose.position.y,
            robot_yaw=yaw_from_quat(q.x, q.y, q.z, q.w),
        )

    def _plan_once(self) -> None:
        if self.map_msg is None or self.occupancy is None:
            self.status.set('failed', 'no map')
            return
        if self.static_edt is None:
            self.status.set('failed', 'no static EDT')
            return
        if self.pose is None or self.pending_goal is None:
            self.status.set('failed', 'missing pose or goal')
            return

        info = self.map_msg.info
        qg = self.pending_goal.pose.orientation
        poses = plan_path(
            self.pose.pose.position.x,
            self.pose.pose.position.y,
            self.pending_goal.pose.position.x,
            self.pending_goal.pose.position.y,
            yaw_from_quat(qg.x, qg.y, qg.z, qg.w),
            self.occupancy,
            self.static_edt,
            info.origin.position.x,
            info.origin.position.y,
            float(info.resolution),
            local=self._local_input(),
            occupied_threshold=self.occupied_thresh,
            clearance_weight=self.clearance_weight,
            min_clearance_m=self.min_clearance_m,
            los_sample_step_m=self.los_step,
        )
        if poses is None:
            self.status.set('failed', 'no path found')
            return

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.map_frame
        for x, y, yaw in poses:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            qx, qy, qz, qw = quat_from_yaw(yaw)
            ps.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
            path.poses.append(ps)

        self.path_pub.publish(path)
        mode = 'replan' if self.use_local else 'plan'
        self.status.set('done', f'{mode} {len(path.poses)} edt-straight waypoints')
        self.get_logger().info(f'{mode}: {len(path.poses)} edt-straight waypoints')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
