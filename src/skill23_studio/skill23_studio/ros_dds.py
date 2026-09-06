"""Fast DDS defaults for Studio (UDP only — works across sudo/root).

ParticipantEntitiesInfo Fast CDR / Bad alloc spam happens when Humble
discovers incompatible ROS 2 peers on Wi‑Fi (Iron/Jazzy/etc. on same
ROS_DOMAIN_ID). Dashboard talks HTTP to the Pi, so ROS stays local by
default (ROS_LOCALHOST_ONLY=1).

Override for multi-machine ROS:
  export STUDIO_DDS_WIFI=1
  # or: export ROS_LOCALHOST_ONLY=0
"""

from __future__ import annotations

import os

# Shared-memory transport breaks when control_server runs as root and other
# nodes as vmx. UDPv4 avoids /dev/shm fastrtps_port* lock errors.
_FASTDDS_UDP = 'UDPv4'


def _localhost_only() -> str:
    """1 = Pi-local only (default). 0 = discover on LAN (Wi‑Fi DDS)."""
    if os.environ.get('STUDIO_DDS_WIFI', '').strip().lower() in (
            '1', 'true', 'yes', 'on'):
        return '0'
    # Respect an explicit user override.
    raw = os.environ.get('ROS_LOCALHOST_ONLY')
    if raw is not None and str(raw).strip() != '':
        return '1' if str(raw).strip().lower() in ('1', 'true', 'yes', 'on') else '0'
    return '1'


def studio_dds_env() -> dict:
    return {
        'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
        'FASTDDS_BUILTIN_TRANSPORTS': _FASTDDS_UDP,
        'ROS_LOCALHOST_ONLY': _localhost_only(),
    }


def apply_studio_dds_env() -> None:
    """Set DDS env vars in the current process before rclpy.init()."""
    for key, value in studio_dds_env().items():
        os.environ[key] = value


def studio_dds_launch_actions():
    """Launch actions: Fast DDS UDP + clear stale DDS shm + localhost default."""
    from launch.actions import ExecuteProcess, LogInfo, SetEnvironmentVariable

    localhost = _localhost_only()
    msg = (
        'Studio DDS: Fast DDS UDPv4, ROS_LOCALHOST_ONLY='
        f'{localhost}'
        + (' (Wi‑Fi discovery ON)' if localhost == '0' else ' (Pi-local only)')
    )
    return [
        ExecuteProcess(
            cmd=['bash', '-c',
                 'rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* '
                 '/dev/shm/*fastrtps* '
                 '/dev/shm/cyclonedds_* /dev/shm/sem.cyclonedds_* 2>/dev/null; true'],
            output='log',
        ),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', _FASTDDS_UDP),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', localhost),
        LogInfo(msg=msg),
    ]
