"""ArUco: align in image, then use depth to drive closer by approach_cm."""

from __future__ import annotations

from typing import Any, Dict

from detectors import aruco, depth_util


def detect(frame, params: Dict[str, Any], depth=None) -> Dict[str, Any]:
    res = aruco.detect(frame, params)
    return depth_util.annotate(res, depth, frame.shape)
