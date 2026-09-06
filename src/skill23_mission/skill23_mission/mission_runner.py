"""Terminal waypoint sequence runner for Nav2.

Loads waypoints + sequences, asks which sequence to run, then sends
NavigateToPose one-by-one (same pattern as Studio). After each goal,
calls waypoint_action() — default wait 3s (edit per WP).

  # Terminal 2 (after bringup + set initial pose)
  ros2 launch skill23_mission mission.launch.py map:=default
"""

from __future__ import annotations

import json
import math
import os
import sys
import time
from typing import Any, Dict, List, Optional

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future


def _yaw_to_quat(yaw: float):
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def _resolve_map_dir(map_arg: str, share: str) -> str:
    map_arg = os.path.expanduser(str(map_arg or '').strip())
    if os.path.isabs(map_arg) and map_arg.endswith('.yaml'):
        return os.path.dirname(map_arg)
    if os.path.isabs(map_arg) and os.path.isdir(map_arg):
        return map_arg
    path = os.path.join(share, 'maps', map_arg)
    if os.path.isdir(path):
        return path
    # Also allow studio_maps/<name> on the Pi
    studio = os.path.expanduser(f'~/skill23_ws/studio_maps/{map_arg}')
    if os.path.isdir(studio):
        return studio
    raise FileNotFoundError(
        f'Map folder not found for map:={map_arg}\n'
        f'Tried: {path} and {studio}')


def _load_waypoints(map_dir: str) -> List[Dict[str, Any]]:
    path = os.path.join(map_dir, 'waypoints.json')
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f'No waypoints.json in {map_dir}\n'
            'Mark WPs in Studio dashboard and Save to Pi, then copy '
            'waypoints.json here (or point map:= at the studio_maps folder).')
    data = json.loads(open(path, encoding='utf-8').read())
    wps = list(data.get('waypoints') or data if isinstance(data, list) else [])
    if not wps:
        raise RuntimeError(f'waypoints.json is empty: {path}')
    return wps


def _load_sequences(map_dir: str, share: str, n_wps: int) -> Dict[str, List[int]]:
    """Return {name: [0-based indices]} from map sequences.json and/or package YAML."""
    out: Dict[str, List[int]] = {}

    # 1) Pi / map folder (Studio uses 0-based indices)
    seq_json = os.path.join(map_dir, 'sequences.json')
    if os.path.isfile(seq_json):
        data = json.loads(open(seq_json, encoding='utf-8').read())
        raw = data.get('sequences') if isinstance(data, dict) else data
        if isinstance(raw, dict):
            for name, idxs in raw.items():
                if isinstance(idxs, list) and idxs:
                    out[str(name)] = [int(i) for i in idxs]

    # 2) Package config/sequences.yaml (1-based by default)
    yaml_path = os.path.join(share, 'config', 'sequences.yaml')
    if os.path.isfile(yaml_path):
        cfg = yaml.safe_load(open(yaml_path, encoding='utf-8')) or {}
        one_based = int(cfg.get('index_base', 1)) == 1
        block = cfg.get('sequences') or {}
        if isinstance(block, dict):
            for key, entry in block.items():
                name = str(key)
                idxs: List[Any] = []
                if isinstance(entry, dict):
                    name = str(entry.get('name') or key)
                    idxs = list(entry.get('waypoints') or entry.get('indices') or [])
                elif isinstance(entry, list):
                    idxs = list(entry)
                if not idxs:
                    # empty → all waypoints
                    out[name] = list(range(n_wps))
                    continue
                nums = [int(i) for i in idxs]
                if one_based:
                    nums = [i - 1 for i in nums]
                out[name] = nums

    if not out:
        out['default'] = list(range(n_wps))
    return out


class MissionRunner(Node):
    def __init__(self) -> None:
        super().__init__('mission_runner')
        self.declare_parameter('map', 'default')
        self.declare_parameter('default_wait_sec', 3.0)
        self.declare_parameter('goal_timeout_sec', 180.0)
        self.declare_parameter('server_wait_sec', 60.0)
        self.declare_parameter('auto_sequence', '')  # skip prompt if set

        share = get_package_share_directory('skill23_mission')
        map_arg = str(self.get_parameter('map').value)
        self._map_dir = _resolve_map_dir(map_arg, share)
        self._waypoints = _load_waypoints(self._map_dir)
        self._sequences = _load_sequences(
            self._map_dir, share, len(self._waypoints))
        cleaned: Dict[str, List[int]] = {}
        n = len(self._waypoints)
        for name, idxs in self._sequences.items():
            valid = [i for i in idxs if 0 <= int(i) < n]
            if valid:
                cleaned[str(name)] = valid
        if not cleaned:
            cleaned['default'] = list(range(n))
        self._sequences = cleaned

        self._default_wait = float(self.get_parameter('default_wait_sec').value)
        self._goal_timeout = float(self.get_parameter('goal_timeout_sec').value)
        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info(
            f'map_dir={self._map_dir}  waypoints={len(self._waypoints)}  '
            f'sequences={list(self._sequences.keys())}')

    def waypoint_action(self, wp_index: int, wp: Dict[str, Any]) -> None:
        """Called after NavigateToPose succeeds. Edit this for custom WP behavior.

        Default: wait N seconds from action.seconds or default_wait_sec (3).
        Examples you can add later:
          - different waits per WP name
          - publish a topic
          - call a service
        """
        action = wp.get('action') if isinstance(wp.get('action'), dict) else {}
        atype = str(action.get('type') or 'wait').lower()
        name = wp.get('name') or f'wp{wp_index + 1}'

        if atype == 'wait' or atype == '':
            sec = action.get('seconds', None)
            if sec is None:
                sec = self._default_wait
            sec = max(0.0, float(sec))
            self.get_logger().info(
                f'[action] {name}: wait {sec:.1f}s  (edit waypoint_action() to customize)')
            if sec > 0:
                deadline = time.time() + sec
                while time.time() < deadline and rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=0.1)
            return

        # Placeholder for future action types — still pause briefly
        self.get_logger().warn(
            f'[action] {name}: unknown type={atype!r} — waiting {self._default_wait:.1f}s')
        deadline = time.time() + self._default_wait
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def _pick_sequence(self) -> List[int]:
        auto = str(self.get_parameter('auto_sequence').value or '').strip()
        names = sorted(self._sequences.keys(), key=lambda s: (s != 'default', s))
        if auto:
            if auto not in self._sequences:
                # allow 1-based number into sorted list
                if auto.isdigit():
                    idx = int(auto) - 1
                    if 0 <= idx < len(names):
                        auto = names[idx]
            if auto not in self._sequences:
                raise RuntimeError(f'Unknown sequence {auto!r}. Have: {names}')
            self.get_logger().info(f'Using auto_sequence={auto}')
            return self._sequences[auto]

        print('\n========== skill23_mission sequences ==========')
        print(f'Map folder: {self._map_dir}')
        print(f'Waypoints:  {len(self._waypoints)}')
        for i, name in enumerate(names, start=1):
            idxs = self._sequences[name]
            chain = ' → '.join(str(j + 1) for j in idxs)
            print(f'  [{i}] {name}  ({len(idxs)} WPs): {chain}')
        print('===============================================')
        while True:
            try:
                raw = input('Enter sequence number (or name): ').strip()
            except EOFError:
                raw = '1'
            if not raw:
                continue
            if raw in self._sequences:
                return self._sequences[raw]
            if raw.isdigit():
                idx = int(raw) - 1
                if 0 <= idx < len(names):
                    return self._sequences[names[idx]]
            print(f'Invalid choice. Pick 1–{len(names)} or a name.')

    def _send_goal(self, wp: Dict[str, Any], label: str) -> bool:
        if not self._nav.wait_for_server(
                timeout_sec=float(self.get_parameter('server_wait_sec').value)):
            self.get_logger().error('navigate_to_pose action server not available')
            return False

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(wp['x'])
        pose.pose.position.y = float(wp['y'])
        pose.pose.position.z = float(wp.get('z') or 0.0)
        pose.pose.orientation = _yaw_to_quat(float(wp.get('yaw') or 0.0))

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(
            f'Navigating {label} → ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}) '
            f'yaw={float(wp.get("yaw") or 0.0):.2f}')

        send_future = self._nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f'{label}: goal rejected')
            return False

        result_future: Future = goal_handle.get_result_async()
        deadline = time.time() + self._goal_timeout
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.2)
            if time.time() > deadline:
                self.get_logger().error(f'{label}: timeout — canceling')
                cancel_fut = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_fut, timeout_sec=5.0)
                return False

        result = result_future.result()
        status = int(getattr(result, 'status', -1))
        # 4 = STATUS_SUCCEEDED in action_msgs
        if status != 4:
            self.get_logger().error(f'{label}: failed status={status}')
            return False
        self.get_logger().info(f'{label}: reached')
        return True

    def run(self) -> int:
        idxs = self._pick_sequence()
        self.get_logger().info(
            f'Running sequence ({len(idxs)} goals): '
            + ' → '.join(str(i + 1) for i in idxs))
        for step, wi in enumerate(idxs, start=1):
            wp = self._waypoints[wi]
            label = f'[{step}/{len(idxs)}] WP{wi + 1}'
            if not self._send_goal(wp, label):
                self.get_logger().error('Mission aborted')
                return 1
            self.waypoint_action(wi, wp)
        self.get_logger().info('Mission complete')
        return 0


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = MissionRunner()
    try:
        code = node.run()
    except KeyboardInterrupt:
        code = 130
    except Exception as exc:
        node.get_logger().error(str(exc))
        code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
