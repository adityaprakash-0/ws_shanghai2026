#!/usr/bin/env python3
"""Map server — exact port of src/map_server_node.cpp."""

from __future__ import annotations

import json
import math
import os
from copy import deepcopy
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from skill23_case_nav.utils import quat_from_yaw


def _load_pgm(path: Path) -> Tuple[int, int, int, List[int]]:
    """Load PGM → (width, height, maxval, occupancy 0..100 row-flipped)."""
    with path.open("rb") as f:
        magic = f.readline().strip()
        if magic not in (b"P5", b"P2"):
            raise ValueError(f"unsupported PGM magic: {magic!r}")

        # Skip comments / blank lines until dimensions
        line = f.readline()
        while line.startswith(b"#") or not line.strip():
            line = f.readline()
        parts = line.split()
        if len(parts) >= 2:
            width, height = int(parts[0]), int(parts[1])
        else:
            width = int(parts[0])
            height = int(f.readline().split()[0])

        maxval = int(f.readline())
        if magic == b"P5":
            raw = f.read(width * height)
            pixels = list(raw)
        else:
            pixels = [int(x) for x in f.read().split()]

    data: List[int] = []
    for row in range(height - 1, -1, -1):
        for col in range(width):
            v = pixels[row * width + col]
            occ = int(round(100.0 * (1.0 - (v / float(maxval)))))
            data.append(max(0, min(100, occ)))
    return width, height, maxval, data


def _parse_map_yaml(yaml_path: Path) -> dict:
    try:
        import yaml

        with yaml_path.open("r") as f:
            return yaml.safe_load(f) or {}
    except Exception:
        meta: dict = {}
        with yaml_path.open("r") as f:
            for line in f:
                if ":" not in line or line.strip().startswith("#"):
                    continue
                k, v = line.split(":", 1)
                meta[k.strip()] = v.strip()
        return meta


class MapServerNode(Node):
    def __init__(self) -> None:
        super().__init__("cn_map_server")
        self.declare_parameter("map", "")
        self.declare_parameter("occupied_thresh", 50)
        self.declare_parameter("map_frame", "map")

        self.map_frame_ = str(self.get_parameter("map_frame").value)
        self.loaded_ = False
        self.map_ = OccupancyGrid()
        self.static_map_ = OccupancyGrid()

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub_ = self.create_publisher(OccupancyGrid, "/map", qos)
        self.meta_pub_ = self.create_publisher(MapMetaData, "/map_metadata", qos)

        self.create_subscription(String, "/case_nav/map/load", self.on_load_cmd, 10)
        self.create_subscription(String, "/case_nav/map/patch", self.on_patch, 10)
        self.create_service(Trigger, "/case_nav/map/reset", self.on_reset)
        # Alias kept for older clients; same behavior as reset (restore static_map).
        self.create_service(Trigger, "/case_nav/map/reload", self.on_reset)

        map_path = str(self.get_parameter("map").value)
        if map_path:
            self.load_map(map_path)

        self.create_timer(2.0, self.republish)

    def load_map(self, yaml_path: str) -> bool:
        expanded = os.path.expanduser(yaml_path)
        path = Path(expanded)
        try:
            meta = _parse_map_yaml(path)
        except Exception as e:
            self.get_logger().error(f"Failed to parse map yaml {expanded}: {e}")
            return False

        image = str(meta.get("image", ""))
        if image and not os.path.isabs(image):
            image = str(path.parent / image)

        try:
            width, height, _maxval, data = _load_pgm(Path(image))
        except Exception as e:
            self.get_logger().error(f"Failed to load PGM {image}: {e}")
            return False

        resolution = float(meta.get("resolution", 0.05))
        negate = int(meta.get("negate", 0) or 0)
        free_thresh = float(meta.get("free_thresh", 0.25))
        occupied_thresh = float(meta.get("occupied_thresh", 0.65))

        origin = meta.get("origin", [0.0, 0.0, 0.0])
        if isinstance(origin, str):
            origin = [float(x) for x in origin.strip("[]").split(",")]
        ox = float(origin[0]) if len(origin) >= 1 else 0.0
        oy = float(origin[1]) if len(origin) >= 2 else 0.0
        oyaw = float(origin[2]) if len(origin) >= 3 else 0.0

        if negate:
            data = [100 - v if v >= 0 else v for v in data]

        classified: List[int] = []
        for v in data:
            if v < 0:
                classified.append(v)
                continue
            p = v / 100.0
            if p >= occupied_thresh:
                classified.append(100)
            elif p <= free_thresh:
                classified.append(0)
            else:
                classified.append(-1)

        grid = OccupancyGrid()
        grid.header.frame_id = self.map_frame_
        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin = Pose()
        grid.info.origin.position.x = ox
        grid.info.origin.position.y = oy
        z, w = quat_from_yaw(oyaw)
        grid.info.origin.orientation.z = z
        grid.info.origin.orientation.w = w
        grid.data = classified

        self.map_ = grid
        self.static_map_ = deepcopy(grid)
        self.loaded_ = True
        self.republish()
        self.get_logger().info(
            f"Loaded map {expanded} {width}x{height} res={resolution:.3f}"
        )
        return True

    def republish(self) -> None:
        if not self.loaded_:
            return
        stamp = self.get_clock().now().to_msg()
        self.map_.header.stamp = stamp
        self.map_.info.map_load_time = stamp
        self.map_pub_.publish(self.map_)
        self.meta_pub_.publish(self.map_.info)

    def on_load_cmd(self, msg: String) -> None:
        if msg.data:
            self.load_map(msg.data)

    def on_reset(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        if not self.loaded_:
            res.success = False
            res.message = "no map loaded"
            return res
        self.map_ = deepcopy(self.static_map_)
        self.republish()
        res.success = True
        res.message = "restored static map"
        self.get_logger().info("Map reset to original static map")
        return res

    def on_patch(self, msg: String) -> None:
        if not self.loaded_:
            return
        try:
            j = json.loads(msg.data)
        except Exception:
            return

        value = int(j.get("value", 100))
        data = list(self.map_.data)
        w = int(self.map_.info.width)
        h = int(self.map_.info.height)

        if isinstance(j.get("cells"), list):
            for c in j["cells"]:
                if not isinstance(c, (list, tuple)) or len(c) < 2:
                    continue
                i, jj = int(c[0]), int(c[1])
                if 0 <= i < w and 0 <= jj < h:
                    data[jj * w + i] = value
            self.map_.data = data
            self.republish()
            return

        pts = []
        if isinstance(j.get("points"), list):
            pts = j["points"]
        elif "x" in j and "y" in j:
            pts = [{"x": j["x"], "y": j["y"]}]
        if not pts:
            return

        res = float(self.map_.info.resolution)
        ox = float(self.map_.info.origin.position.x)
        oy = float(self.map_.info.origin.position.y)
        for p in pts:
            i = int((float(p["x"]) - ox) / res)
            jj = int((float(p["y"]) - oy) / res)
            if 0 <= i < w and 0 <= jj < h:
                data[jj * w + i] = value
        self.map_.data = data
        self.republish()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = MapServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
