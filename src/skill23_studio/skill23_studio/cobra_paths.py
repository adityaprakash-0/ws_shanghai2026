"""Resolve per-robot Cobra-align yaml."""

from __future__ import annotations

import os
from typing import Optional

from ament_index_python.packages import get_package_share_directory

_ROBOTS = ('allterrain', 'grip', 'mecanum', 'xdrive')


def cobra_yaml_for_robot(robot: str, share: Optional[str] = None) -> str:
    """Return ``config/cobra/wp_cobra_<robot>.yaml`` under skill23_studio share."""
    if robot not in _ROBOTS:
        raise ValueError(
            f"Unknown robot '{robot}'. Use: allterrain | grip | mecanum | xdrive")
    if not share:
        share = get_package_share_directory('skill23_studio')
    path = os.path.join(share, 'config', 'cobra', f'wp_cobra_{robot}.yaml')
    if not os.path.isfile(path):
        raise FileNotFoundError(f'Missing cobra config: {path}')
    return path
