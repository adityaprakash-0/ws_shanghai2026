"""Resolve Detect+Align yaml (shared OMS config) and the NCNN YOLOv8n model dir."""

from __future__ import annotations

import os
from typing import Optional

from ament_index_python.packages import get_package_share_directory

_ROBOTS = ('allterrain', 'grip', 'mecanum', 'xdrive')


def detect_align_yaml_for_robot(robot: str, share: Optional[str] = None) -> str:
    """OMS is the same on every robot — always load wp_detect_align.yaml."""
    if robot not in _ROBOTS:
        raise ValueError(
            f"Unknown robot '{robot}'. Use: allterrain | grip | mecanum | xdrive")
    if not share:
        share = get_package_share_directory('skill23_studio')
    path = os.path.join(share, 'config', 'wp_detect_align.yaml')
    if not os.path.isfile(path):
        raise FileNotFoundError(f'Missing detect-align config: {path}')
    return path


def yolov8n_ncnn_dir(share: Optional[str] = None) -> str:
    """Prefer installed share, then the workspace source tree."""
    candidates = []
    if share:
        candidates.append(os.path.join(share, 'models', 'yolov8n_ncnn'))
    try:
        pkg = get_package_share_directory('skill23_studio')
        candidates.append(os.path.join(pkg, 'models', 'yolov8n_ncnn'))
    except Exception:
        pass
    ws = os.path.expanduser('~/skill23_ws/src/skill23_studio/models/yolov8n_ncnn')
    candidates.append(ws)
    for path in candidates:
        param = os.path.join(path, 'model.ncnn.param')
        binary = os.path.join(path, 'model.ncnn.bin')
        if os.path.isfile(param) and os.path.isfile(binary):
            return path
    return candidates[0]
