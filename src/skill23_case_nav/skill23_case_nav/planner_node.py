#!/usr/bin/env python3
"""Planner — exact port of src/planner_node.cpp."""

from __future__ import annotations

import heapq
import json
import math
from typing import Dict, List, Optional, Set, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from skill23_case_nav.utils import (
    normalize_heading,
    parse_goal_json,
    yaw_from_quat,
    yaw_to_heading,
)

Cell = Tuple[int, int]
State = Tuple[int, int, int]  # (i, j, heading)


class PlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("cn_planner")

        self.declare_parameter("grid_cell_m", 0.12)
        self.declare_parameter("occupied_thresh", 50)
        self.declare_parameter("goal_xy_tol", 0.15)
        self.declare_parameter("turn_penalty", 80.0)
        # Prefer corridor center over shortest path (higher = more centering).
        self.declare_parameter("clearance_weight", 60.0)
        # Robot half-width used for footprint inflation (meters).
        self.declare_parameter("robot_width", 0.40)
        self.declare_parameter("robot_length", 0.40)
        # Extra safety padding beyond half-footprint (meters).
        self.declare_parameter("inflate_margin_m", 0.04)

        self.grid_cell_m_ = float(self.get_parameter("grid_cell_m").value)
        self.occupied_thresh_ = int(self.get_parameter("occupied_thresh").value)
        self.goal_xy_tol_ = float(self.get_parameter("goal_xy_tol").value)
        self.turn_penalty_ = float(self.get_parameter("turn_penalty").value)
        self.clearance_weight_ = float(self.get_parameter("clearance_weight").value)
        half_w = 0.5 * float(self.get_parameter("robot_width").value)
        half_l = 0.5 * float(self.get_parameter("robot_length").value)
        margin = float(self.get_parameter("inflate_margin_m").value)
        # Conservative radius: half of the longer side + margin.
        self.inflate_m_ = max(half_w, half_l) + margin

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map", self.on_map, qos)
        self.create_subscription(PoseStamped, "/case_nav/pose", self.on_pose, 10)
        self.create_subscription(String, "/case_nav/goal", self.on_goal, 10)
        self.create_subscription(String, "/case_nav/replan", self.on_replan, 10)

        self.plan_pub_ = self.create_publisher(String, "/case_nav/plan", 10)
        self.path_pub_ = self.create_publisher(Path, "/case_nav/path", 10)
        self.status_pub_ = self.create_publisher(String, "/case_nav/planner_status", 10)

        self.map_: Optional[OccupancyGrid] = None
        self.pose_: Optional[PoseStamped] = None
        self.goals_: List[Tuple[float, float, float]] = []
        self.plan_: List[dict] = []
        self.clearance_: List[float] = []
        self.clearance_valid_ = False

    def on_map(self, msg: OccupancyGrid) -> None:
        self.map_ = msg
        self.clearance_ = []
        self.clearance_valid_ = False

    def on_pose(self, msg: PoseStamped) -> None:
        self.pose_ = msg

    def on_goal(self, msg: String) -> None:
        goals = parse_goal_json(msg.data)
        if goals is None:
            self.get_logger().warn("Bad goal JSON")
            return
        self.goals_ = [(g["x"], g["y"], g["yaw"]) for g in goals]
        self.get_logger().info(f"Received {len(self.goals_)} goal(s)")
        self.plan()

    def on_replan(self, _msg: String) -> None:
        if self.goals_:
            self.plan()

    def step(self) -> int:
        if self.map_ is None:
            return 1
        return max(1, int(round(self.grid_cell_m_ / self.map_.info.resolution)))

    def world_to_cell(self, x: float, y: float) -> Optional[Cell]:
        if self.map_ is None:
            return None
        st = self.step()
        i = int((x - self.map_.info.origin.position.x) / self.map_.info.resolution)
        j = int((y - self.map_.info.origin.position.y) / self.map_.info.resolution)
        i = (i // st) * st
        j = (j // st) * st
        if (
            i < 0
            or j < 0
            or i >= int(self.map_.info.width)
            or j >= int(self.map_.info.height)
        ):
            return None
        return (i, j)

    def cell_to_world(self, cell: Cell) -> Tuple[float, float]:
        assert self.map_ is not None
        st = self.step()
        x = (
            self.map_.info.origin.position.x
            + (cell[0] + st * 0.5) * self.map_.info.resolution
        )
        y = (
            self.map_.info.origin.position.y
            + (cell[1] + st * 0.5) * self.map_.info.resolution
        )
        return (x, y)

    def map_cell_free(self, i: int, j: int) -> bool:
        if self.map_ is None:
            return False
        w = int(self.map_.info.width)
        h = int(self.map_.info.height)
        if i < 0 or j < 0 or i >= w or j >= h:
            return False
        v = int(self.map_.data[j * w + i])
        return v >= 0 and v < self.occupied_thresh_

    def inflate_cells(self) -> int:
        """Footprint radius in map cells (not planner step cells)."""
        if self.map_ is None:
            return 1
        res = float(self.map_.info.resolution)
        return max(1, int(math.ceil(self.inflate_m_ / max(res, 1e-6))))

    def is_free(self, cell: Cell) -> bool:
        """True if the robot footprint centered on this planner cell is free."""
        st = self.step()
        inflate = self.inflate_cells()
        # Check the full robot square covering the planner cell + radius.
        for jj in range(cell[1] - inflate, cell[1] + st + inflate):
            for ii in range(cell[0] - inflate, cell[0] + st + inflate):
                if not self.map_cell_free(ii, jj):
                    return False
        return True

    def cell_clearance_cells(self, i: int, j: int) -> float:
        if self.map_ is None or not self.clearance_:
            return 0.0
        w = int(self.map_.info.width)
        h = int(self.map_.info.height)
        st = self.step()
        best = 0.0
        for jj in range(j, min(j + st, h)):
            for ii in range(i, min(i + st, w)):
                best = max(best, self.clearance_[jj * w + ii])
        return best

    def build_clearance(self) -> None:
        if self.map_ is None or self.clearance_valid_:
            return
        w = int(self.map_.info.width)
        h = int(self.map_.info.height)
        self.clearance_ = [0.0] * (w * h)
        q: List[int] = []
        for j in range(h):
            for i in range(w):
                idx = j * w + i
                if not self.map_cell_free(i, j):
                    self.clearance_[idx] = 0.0
                    q.append(idx)
                else:
                    self.clearance_[idx] = float(w + h)
        head = 0
        dx = (1, -1, 0, 0)
        dy = (0, 0, 1, -1)
        while head < len(q):
            idx = q[head]
            head += 1
            cx = idx % w
            cy = idx // w
            cd = self.clearance_[idx]
            for k in range(4):
                nx = cx + dx[k]
                ny = cy + dy[k]
                if nx < 0 or ny < 0 or nx >= w or ny >= h:
                    continue
                nidx = ny * w + nx
                nd = cd + 1.0
                if nd < self.clearance_[nidx]:
                    self.clearance_[nidx] = nd
                    q.append(nidx)
        self.clearance_valid_ = True

    def cell_clearance_cost(self, i: int, j: int) -> float:
        """High cost near walls → A* prefers corridor center even if longer."""
        best = self.cell_clearance_cells(i, j)
        # Quadratic-ish: walls dominate; center is nearly free.
        return float(self.clearance_weight_ / ((best + 0.5) ** 2))

    def resolve_free_cell(self, cell: Optional[Cell]) -> Optional[Cell]:
        if cell is None:
            return None
        if self.is_free(cell):
            return cell
        st = self.step()
        dirs = ((st, 0), (-st, 0), (0, st), (0, -st))
        for d in dirs:
            n = (cell[0] + d[0], cell[1] + d[1])
            if self.is_free(n):
                return n
        return cell

    @staticmethod
    def heading_for_delta(di: int, dj: int) -> int:
        if di > 0 and dj == 0:
            return 0
        if di < 0 and dj == 0:
            return 2
        if dj > 0 and di == 0:
            return 1
        return 3

    def astar(
        self, start: Cell, goal: Cell, start_heading: int
    ) -> List[Cell]:
        self.build_clearance()
        st = self.step()
        moves = ((st, 0), (0, st), (-st, 0), (0, -st))

        def heuristic(s: State) -> float:
            return float(abs(s[0] - goal[0]) + abs(s[1] - goal[1]))

        start_state: State = (
            start[0],
            start[1],
            normalize_heading(start_heading),
        )
        open_heap: List[Tuple[float, int, State]] = []
        tie = 0
        heapq.heappush(open_heap, (heuristic(start_state), tie, start_state))
        came_from: Dict[State, State] = {}
        gscore: Dict[State, float] = {start_state: 0.0}
        closed: Set[State] = set()

        while open_heap:
            _f, _t, cur = heapq.heappop(open_heap)
            if cur in closed:
                continue
            closed.add(cur)
            if (cur[0], cur[1]) == goal:
                path: List[Cell] = []
                s = cur
                while True:
                    path.append((s[0], s[1]))
                    if s not in came_from:
                        break
                    s = came_from[s]
                path.reverse()
                return path

            ci, cj, ch = cur
            gcur = gscore[cur]

            for nh in range(4):
                if nh != ch:
                    ns: State = (ci, cj, nh)
                    ng = gcur + self.turn_penalty_
                    if ns not in gscore or ng < gscore[ns]:
                        gscore[ns] = ng
                        came_from[ns] = cur
                        tie += 1
                        heapq.heappush(
                            open_heap, (ng + heuristic(ns), tie, ns)
                        )

            di, dj = moves[ch]
            nb: Cell = (ci + di, cj + dj)
            if not self.is_free(nb):
                continue
            ns = (nb[0], nb[1], ch)
            # Base step cheap; clearance dominates so path hugs corridor midline.
            move_cost = 1.0 + self.cell_clearance_cost(nb[0], nb[1])
            ng = gcur + move_cost
            if ns not in gscore or ng < gscore[ns]:
                gscore[ns] = ng
                came_from[ns] = cur
                tie += 1
                heapq.heappush(open_heap, (ng + heuristic(ns), tie, ns))
        return []

    def pull_path_to_center(self, cells: List[Cell]) -> List[Cell]:
        """At turn corners, pick the free 4-neighbor with max clearance (stay connected)."""
        if len(cells) < 3 or self.map_ is None:
            return cells
        self.build_clearance()
        st = self.step()

        def adj(a: Cell, b: Cell) -> bool:
            return abs(a[0] - b[0]) + abs(a[1] - b[1]) == st

        out = list(cells)
        for k in range(1, len(out) - 1):
            d_in = (out[k][0] - out[k - 1][0], out[k][1] - out[k - 1][1])
            d_out = (out[k + 1][0] - out[k][0], out[k + 1][1] - out[k][1])
            if d_in == d_out:
                continue  # straight run — A* clearance already biases center
            best = out[k]
            best_c = self.cell_clearance_cells(out[k][0], out[k][1])
            for di, dj in ((st, 0), (-st, 0), (0, st), (0, -st), (0, 0)):
                cand = (out[k][0] + di, out[k][1] + dj) if (di or dj) else out[k]
                if not self.is_free(cand):
                    continue
                if not (adj(out[k - 1], cand) and adj(cand, out[k + 1])):
                    continue
                c = self.cell_clearance_cells(cand[0], cand[1])
                if c > best_c:
                    best_c = c
                    best = cand
            out[k] = best
        return out

    def append_final_heading(
        self, prims: List[dict], goal_yaw: float, arrival_heading: int
    ) -> List[dict]:
        heading = arrival_heading
        for p in reversed(prims):
            if p.get("type") == "DRIVE" and "heading" in p:
                heading = int(p["heading"])
                break
        want = yaw_to_heading(goal_yaw)
        diff = (want - heading + 4) % 4
        if diff == 1:
            prims.append({"type": "TURN_LEFT_90", "immediate": True})
        elif diff == 3:
            prims.append({"type": "TURN_RIGHT_90", "immediate": True})
        elif diff == 2:
            prims.append({"type": "TURN_LEFT_90", "immediate": True})
            prims.append({"type": "TURN_LEFT_90", "immediate": True})
        return prims

    def heading_after(self, prims: List[dict], start_heading: int) -> int:
        heading = normalize_heading(start_heading)
        for p in prims:
            ptype = str(p.get("type", ""))
            if ptype == "DRIVE" and "heading" in p:
                heading = int(p["heading"])
            elif ptype == "TURN_LEFT_90":
                heading = (heading + 1) % 4
            elif ptype == "TURN_RIGHT_90":
                heading = (heading + 3) % 4
        return heading

    def cells_to_primitives(
        self, cells: List[Cell], start_heading: int
    ) -> List[dict]:
        if len(cells) < 2:
            return []
        prims: List[dict] = []
        heading = normalize_heading(start_heading)
        i = 0
        while i + 1 < len(cells):
            run_start = i
            di = cells[i + 1][0] - cells[i][0]
            dj = cells[i + 1][1] - cells[i][1]
            want = self.heading_for_delta(di, dj)
            diff = (want - heading + 4) % 4
            if diff == 1:
                prims.append({"type": "TURN_LEFT_90", "immediate": True})
                heading = want
            elif diff == 3:
                prims.append({"type": "TURN_RIGHT_90", "immediate": True})
                heading = want
            elif diff == 2:
                prims.append({"type": "TURN_LEFT_90", "immediate": True})
                prims.append({"type": "TURN_LEFT_90", "immediate": True})
                heading = want
            while i + 1 < len(cells):
                ndi = cells[i + 1][0] - cells[i][0]
                ndj = cells[i + 1][1] - cells[i][1]
                if ndi != di or ndj != dj:
                    break
                i += 1
            x0, y0 = self.cell_to_world(cells[run_start])
            x1, y1 = self.cell_to_world(cells[i])
            dist = math.hypot(x1 - x0, y1 - y0)
            prims.append(
                {
                    "type": "DRIVE",
                    "distance_m": round(max(dist, 0.05) * 1000.0) / 1000.0,
                    "heading": heading,
                    "target_x": round(x1 * 1000.0) / 1000.0,
                    "target_y": round(y1 * 1000.0) / 1000.0,
                }
            )
        return prims

    def publish_status(self, ok: bool, message: str) -> None:
        msg = String()
        msg.data = json.dumps({"ok": ok, "message": message})
        self.status_pub_.publish(msg)

    def publish_plan(self) -> None:
        msg = String()
        msg.data = json.dumps({"primitives": self.plan_})
        self.plan_pub_.publish(msg)

    def publish_path(self, cells: List[Cell]) -> None:
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        for c in cells:
            ps = PoseStamped()
            ps.header = path.header
            x, y = self.cell_to_world(c)
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub_.publish(path)

    def plan(self) -> None:
        if self.map_ is None:
            self.publish_status(False, "no map")
            return
        if self.pose_ is None:
            self.publish_status(False, "no pose (set pose first)")
            return
        if not self.goals_:
            self.publish_status(False, "no goal")
            return

        px = self.pose_.pose.position.x
        py = self.pose_.pose.position.y
        yaw = yaw_from_quat(
            self.pose_.pose.orientation.z, self.pose_.pose.orientation.w
        )
        start = self.resolve_free_cell(self.world_to_cell(px, py))
        if start is None:
            self.publish_status(False, "start off map")
            return

        all_prims: List[dict] = []
        all_cells: List[Cell] = [start]
        heading = yaw_to_heading(yaw)
        cur = start

        for gi, g in enumerate(self.goals_):
            goal = self.resolve_free_cell(self.world_to_cell(g[0], g[1]))
            if goal is None:
                self.publish_status(False, f"goal {gi + 1} off map")
                return
            path_cells = self.astar(cur, goal, heading)
            if not path_cells:
                self.plan_ = []
                self.publish_plan()
                self.publish_status(False, f"no path to goal {gi + 1}")
                return
            # Push path (esp. turn corners) toward corridor center.
            path_cells = self.pull_path_to_center(path_cells)
            seg = self.cells_to_primitives(path_cells, heading)
            seg = self.append_final_heading(seg, g[2], heading)
            heading = self.heading_after(seg, heading)
            all_prims.extend(seg)
            if all_cells:
                all_cells.extend(path_cells[1:])
            else:
                all_cells = path_cells
            cur = goal

        self.plan_ = all_prims
        self.publish_plan()
        self.publish_path(all_cells)
        msg = (
            f"{len(all_prims)} primitives, {len(all_cells)} cells, "
            f"{len(self.goals_)} goal(s)"
        )
        self.publish_status(True, msg)
        self.get_logger().info(msg)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
