#!/usr/bin/env python3
"""World model — load a map yaml+pgm once, publish /map + EDT + ready.

Reads resolution/origin/thresholds from the map yaml, builds an OccupancyGrid
at that resolution (width×height from the PGM), computes a clearance distance
field (EDT), then publishes:

  /map                      OccupancyGrid   (latched)
  /map_metadata             MapMetaData     (latched)
  /custom_nav/static_edt    OccupancyGrid   (latched, clearance in cells)
  /custom_nav/world/ready   Bool            (latched True when done)

Optional: String on /custom_nav/map/load with a new yaml path to reload.
There is no subscription to /map — this node owns the map.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from skill23_studio.nav.node_status import NodeStatusPub
from skill23_studio.nav.utils import compute_edt, quat_from_yaw


def _load_pgm(path: Path) -> Tuple[int, int, List[int]]:
    """Load P5/P2 PGM → (width, height, raw occupancy 0..100, ROS row order)."""
    with path.open('rb') as f:
        magic = f.readline().strip()
        if magic not in (b'P5', b'P2'):
            raise ValueError(f'unsupported PGM magic: {magic!r}')

        line = f.readline()
        while line.startswith(b'#') or not line.strip():
            line = f.readline()
        parts = line.split()
        width, height = int(parts[0]), int(parts[1])
        maxval = int(f.readline())
        if magic == b'P5':
            raw = f.read(width * height)
            pixels = list(raw)
        else:
            pixels = [int(x) for x in f.read().split()]

    # PGM row 0 = top; OccupancyGrid row 0 = bottom → flip Y
    data: List[int] = []
    for row in range(height - 1, -1, -1):
        for col in range(width):
            v = pixels[row * width + col]
            occ = int(round(100.0 * (1.0 - (v / float(maxval)))))
            data.append(max(0, min(100, occ)))
    return width, height, data


def _parse_map_yaml(yaml_path: Path) -> dict:
    try:
        import yaml
        with yaml_path.open('r') as f:
            return yaml.safe_load(f) or {}
    except Exception:
        meta: dict = {}
        with yaml_path.open('r') as f:
            for line in f:
                if ':' not in line or line.strip().startswith('#'):
                    continue
                k, v = line.split(':', 1)
                meta[k.strip()] = v.strip()
        return meta


def _as_float_list(origin) -> List[float]:
    if isinstance(origin, str):
        return [float(x) for x in origin.strip('[]').split(',')]
    if isinstance(origin, (list, tuple)):
        return [float(x) for x in origin]
    return [0.0, 0.0, 0.0]


class WorldModelNode(Node):
    def __init__(self) -> None:
        super().__init__('cn_world_model')
        self.declare_parameter('map', '')
        self.declare_parameter('map_frame', 'map')
        # Cell value ≥ this (0–100) counts as occupied for the EDT
        self.declare_parameter('occupied_thresh', 50)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.occupied_thresh = int(self.get_parameter('occupied_thresh').value)

        self.map_msg: Optional[OccupancyGrid] = None
        self.edt_m: Optional[np.ndarray] = None  # clearance meters, HxW
        self.ready = False
        self.status = NodeStatusPub(self, 'world_model')
        self.status.set('starting', 'loading map')

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', latched)
        self.meta_pub = self.create_publisher(MapMetaData, '/map_metadata', latched)
        self.edt_pub = self.create_publisher(
            OccupancyGrid, '/custom_nav/static_edt', latched)
        self.ready_pub = self.create_publisher(
            Bool, '/custom_nav/world/ready', latched)

        # Optional reload from another yaml path (dashboard); still file-based
        self.create_subscription(String, '/custom_nav/map/load', self._on_load_cmd, 10)

        map_path = str(self.get_parameter('map').value).strip()
        if not map_path:
            self.get_logger().error(
                'No map file given. Launch with map:=/path/to/map.yaml')
            self._publish_ready(False)
            self.status.set('failed', 'no map file configured')
            return

        if not self.load_map(map_path):
            self._publish_ready(False)
            self.status.set('failed', f'could not load map: {map_path}')

    def load_map(self, yaml_path: str) -> bool:
        """Load yaml+pgm → occupancy + EDT → publish → ready."""
        expanded = os.path.expanduser(yaml_path)
        path = Path(expanded)
        if not path.is_file():
            self.get_logger().error(f'Map yaml not found: {expanded}')
            return False

        try:
            meta = _parse_map_yaml(path)
        except Exception as exc:
            self.get_logger().error(f'Failed to parse map yaml {expanded}: {exc}')
            return False

        image = str(meta.get('image', '') or '')
        if not image:
            self.get_logger().error(f'No image: key in {expanded}')
            return False
        if not os.path.isabs(image):
            image = str(path.parent / image)
        if not Path(image).is_file():
            self.get_logger().error(f'Map PGM not found: {image}')
            return False

        try:
            width, height, raw = _load_pgm(Path(image))
        except Exception as exc:
            self.get_logger().error(f'Failed to load PGM {image}: {exc}')
            return False

        # Resolution / origin / thresholds come from the yaml — grid size is
        # (width × height) cells at that resolution (meters/cell).
        resolution = float(meta.get('resolution', 0.05))
        free_thresh = float(meta.get('free_thresh', 0.25))
        occupied_thresh = float(meta.get('occupied_thresh', 0.65))
        origin = _as_float_list(meta.get('origin', [0.0, 0.0, 0.0]))
        ox = origin[0] if len(origin) >= 1 else 0.0
        oy = origin[1] if len(origin) >= 2 else 0.0
        oyaw = origin[2] if len(origin) >= 3 else 0.0

        classified: List[int] = []
        for v in raw:
            p = v / 100.0
            if p >= occupied_thresh:
                classified.append(100)
            elif p <= free_thresh:
                classified.append(0)
            else:
                classified.append(-1)

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info.map_load_time = msg.header.stamp
        msg.info.resolution = resolution
        msg.info.width = width
        msg.info.height = height
        msg.info.origin = Pose()
        msg.info.origin.position.x = ox
        msg.info.origin.position.y = oy
        qx, qy, qz, qw = quat_from_yaw(oyaw)
        msg.info.origin.orientation.x = qx
        msg.info.origin.orientation.y = qy
        msg.info.origin.orientation.z = qz
        msg.info.origin.orientation.w = qw
        msg.data = classified

        occ = np.array(classified, dtype=np.int16).reshape(height, width)
        self.edt_m = compute_edt(occ, resolution, self.occupied_thresh)
        self.map_msg = msg

        self._publish_map()
        self._publish_edt()
        self._publish_ready(True)
        self.status.set(
            'ready',
            f'map {width}x{height} @ {resolution:.4f}m')

        world_w = width * resolution
        world_h = height * resolution
        self.get_logger().info(
            f'World ready: {expanded} → {width}×{height} cells @ '
            f'{resolution:.4f} m/cell ({world_w:.2f}×{world_h:.2f} m) '
            f'origin=({ox:.3f},{oy:.3f})')
        return True

    def _publish_map(self) -> None:
        assert self.map_msg is not None
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.map_msg)
        self.meta_pub.publish(self.map_msg.info)

    def _publish_edt(self) -> None:
        """Publish clearance field as OccupancyGrid (same size/res/origin).

        Cell value = clearance in cells (meters / resolution), capped at 100.
        Occupied cells = 100. Unknown = -1.
        """
        assert self.map_msg is not None and self.edt_m is not None
        res = float(self.map_msg.info.resolution)
        w = self.map_msg.info.width
        h = self.map_msg.info.height

        grid = OccupancyGrid()
        grid.header = self.map_msg.header
        grid.info = self.map_msg.info
        out: List[int] = []
        for y in range(h):
            for x in range(w):
                v = int(self.map_msg.data[y * w + x])
                if v < 0:
                    out.append(-1)
                elif v >= self.occupied_thresh:
                    out.append(100)
                else:
                    cells = int(self.edt_m[y, x] / res) if res > 0 else 0
                    out.append(min(100, max(0, cells)))
        grid.data = out
        self.edt_pub.publish(grid)

    def _publish_ready(self, ok: bool) -> None:
        self.ready = bool(ok)
        msg = Bool()
        msg.data = self.ready
        self.ready_pub.publish(msg)

    def _on_load_cmd(self, msg: String) -> None:
        path = msg.data.strip()
        if path:
            self.get_logger().info(f'Reload requested: {path}')
            if not self.load_map(path):
                self._publish_ready(False)
                self.status.set('failed', f'map reload failed: {path}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WorldModelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
