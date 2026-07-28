"""Utility helpers matching skill23_case_nav C++ utils.hpp / json_utils.hpp."""

from __future__ import annotations

import json
import math
from typing import Any, Dict, List, Optional, Tuple


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_from_quat(z: float, w: float) -> float:
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def quat_from_yaw(yaw: float) -> Tuple[float, float]:
    """Return (z, w) matching C++ quat_from_yaw."""
    z = math.sin(yaw * 0.5)
    w = math.cos(yaw * 0.5)
    return z, w


def normalize_heading(h: int) -> int:
    return ((h % 4) + 4) % 4


def yaw_to_heading(yaw: float) -> int:
    deg = yaw * 180.0 / math.pi
    if deg >= -45.0 and deg < 45.0:
        return 0
    if deg >= 45.0 and deg < 135.0:
        return 1
    if deg >= 135.0 or deg < -135.0:
        return 2
    return 3


def heading_to_yaw(h: int) -> float:
    yaws = (0.0, math.pi / 2.0, math.pi, -math.pi / 2.0)
    return yaws[normalize_heading(h)]


def parse_goal_json(data: str) -> Optional[List[Dict[str, float]]]:
    """Parse goal JSON into a list of waypoint dicts with x, y, yaw.

    Matches C++ parse_goal_json / Waypoint struct.
    """
    try:
        j: Any = json.loads(data)
        out: List[Dict[str, float]] = []
        if (
            isinstance(j, dict)
            and "waypoints" in j
            and isinstance(j["waypoints"], list)
            and len(j["waypoints"]) > 0
        ):
            for wp in j["waypoints"]:
                out.append(
                    {
                        "x": float(wp["x"]),
                        "y": float(wp["y"]),
                        "yaw": float(wp.get("yaw", 0.0)),
                    }
                )
            return out
        if isinstance(j, dict) and "x" in j and "y" in j:
            out.append(
                {
                    "x": float(j["x"]),
                    "y": float(j["y"]),
                    "yaw": float(j.get("yaw", 0.0)),
                }
            )
            return out
    except Exception:
        return None
    return None
