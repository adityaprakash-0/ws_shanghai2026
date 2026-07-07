import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable

_launch_dir = os.path.dirname(os.path.realpath(__file__))
if _launch_dir not in sys.path:
    sys.path.append(_launch_dir)
from _navigation_pi_common import build_pi_bringup_launch_description


def generate_launch_description():
    studica_pkg_share = get_package_share_directory('studica_control')
    allterrain_pkg_share = get_package_share_directory('allterrain')
    mecanum_pkg_share = get_package_share_directory('mecanum')

    ld = build_pi_bringup_launch_description(
        robot_name='mecanum',
        default_params=os.path.join(studica_pkg_share, 'config', 'params_mecanum.yaml'),
        urdf_file=os.path.join(mecanum_pkg_share, 'urdf', 'mecanum_robot.urdf'),
        ekf_config_file=os.path.join(mecanum_pkg_share, 'config', 'ekf_mecanum.yaml'),
        gate_params_file=os.path.join(allterrain_pkg_share, 'config', 'confidence_scan_gate.yaml'),
        drive_package='mecanum',
        drive_executable='mecanum_node',
        drive_node_name='mecanum_node',
        enable_lidar_shutdown_handlers=True,
    )

    ld.add_action(SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'))
    return ld
