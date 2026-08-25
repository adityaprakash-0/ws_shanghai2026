"""Resolve per-robot EKF yaml paths for Skill23 Studio stacks."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory


def ekf_yaml_for_robot(robot: str, *, navigation: bool = False) -> str:
    """Return EKF params file for *robot*.

    Mapping / localization (``navigation=False``):
      xdrive, mecanum → full holonomic ``ekf_<robot>.yaml``
      grip, allterrain → ``ekf_<robot>.yaml``

    Deploy navigation (``navigation=True``):
      xdrive, mecanum → diff-drive ``ekf_<robot>_diff.yaml``
      grip, allterrain → ``ekf_<robot>.yaml``
    """
    if robot not in ('allterrain', 'grip', 'mecanum', 'xdrive'):
        raise ValueError(
            f"Unknown robot '{robot}'. Use: allterrain | grip | mecanum | xdrive")

    robot_share = get_package_share_directory(robot)
    if navigation and robot in ('xdrive', 'mecanum'):
        name = f'ekf_{robot}_diff.yaml'
    else:
        name = f'ekf_{robot}.yaml'

    path = os.path.join(robot_share, 'config', name)
    if not os.path.isfile(path):
        raise FileNotFoundError(f'Missing EKF config: {path}')
    return path
