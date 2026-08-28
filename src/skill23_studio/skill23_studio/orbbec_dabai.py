"""Orbbec Gemini E launch arguments (workspace overlay).

Color/depth stay *configured* (enable_*=true) so /camera/toggle_color and
/camera/toggle_depth exist. start_streams_on_init=false keeps USB idle
until a toggle ON (Detect+Align or Camera ON).
Prefer ~/skill23_ws/install/orbbec_camera over /opt/ros/humble.
"""

from __future__ import annotations

import os
from typing import Dict, List, Optional

ORBBEC_LAUNCH_FILE = 'gemini_e.launch.py'
_WS_INSTALL = os.path.expanduser('~/skill23_ws/install/orbbec_camera')

# Native Gemini E 640x360. Device has NO 3fps mode — lowest MJPG is 5fps.
# enable_color/depth MUST be true so toggle_* services exist.
DABAI_LAUNCH_ARGS: Dict[str, str] = {
    'enable_color': 'true',
    'enable_depth': 'true',
    'enable_ir': 'false',
    'enable_ldp': 'false',
    'enable_point_cloud': 'false',
    'enable_colored_point_cloud': 'false',
    'enable_d2c_viewer': 'false',
    'enable_heartbeat': 'false',
    'enable_publish_extrinsic': 'false',
    'enable_soft_filter': 'false',
    'diagnostic_period': '0.0',
    'color_width': '640',
    'color_height': '360',
    'color_fps': '5',
    'color_format': 'MJPG',
    'color_qos': 'default',
    'depth_width': '640',
    'depth_height': '360',
    'depth_fps': '5',
    'publish_tf': 'false',
    'tf_publish_rate': '0.0',
    'connection_delay': '500',
    'start_streams_on_init': 'false',
}

ORBBEC_SHM_LOCKS = (
    '/dev/shm/orbbec_device_lock',
    '/dev/shm/sem.orbbec_device_lock',
)


def prefer_workspace_orbbec() -> str:
    """Put workspace overlay first so component_container loads our .so."""
    if not os.path.isdir(_WS_INSTALL):
        return 'humble'
    lib = os.path.join(_WS_INSTALL, 'lib')

    def _prepend(env: str, path: str) -> None:
        cur = os.environ.get(env, '')
        parts = [p for p in cur.split(':') if p and p != path]
        os.environ[env] = path + (':' + ':'.join(parts) if parts else '')

    _prepend('AMENT_PREFIX_PATH', _WS_INSTALL)
    _prepend('CMAKE_PREFIX_PATH', _WS_INSTALL)
    _prepend('COLCON_PREFIX_PATH', _WS_INSTALL)
    if os.path.isdir(lib):
        _prepend('LD_LIBRARY_PATH', lib)
    return _WS_INSTALL


def orbbec_launch_path() -> str:
    prefer_workspace_orbbec()
    local = os.path.join(
        _WS_INSTALL, 'share', 'orbbec_camera', 'launch', ORBBEC_LAUNCH_FILE)
    if os.path.isfile(local):
        return local
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(
        get_package_share_directory('orbbec_camera'), 'launch', ORBBEC_LAUNCH_FILE)


def wake_orbbec_usb() -> str:
    """Keep Gemini E from USB autosuspend. Does not turn USB2 into USB3."""
    notes: List[str] = []
    sysfs = '/sys/bus/usb/devices'
    if not os.path.isdir(sysfs):
        return 'no usb sysfs'
    for name in os.listdir(sysfs):
        base = os.path.join(sysfs, name)
        vid_path = os.path.join(base, 'idVendor')
        try:
            with open(vid_path, 'r', encoding='ascii') as f:
                vid = f.read().strip().lower()
        except OSError:
            continue
        if vid != '2bc5':
            continue
        for ctrl in (
                os.path.join(base, 'power', 'control'),
                os.path.join(base, 'power', 'autosuspend'),
                os.path.join(base, 'power', 'autosuspend_delay_ms')):
            try:
                if ctrl.endswith('control'):
                    with open(ctrl, 'w', encoding='ascii') as f:
                        f.write('on')
                elif ctrl.endswith('autosuspend'):
                    with open(ctrl, 'w', encoding='ascii') as f:
                        f.write('-1')
                else:
                    with open(ctrl, 'w', encoding='ascii') as f:
                        f.write('-1')
            except OSError:
                pass
        notes.append(name)
    return ('autosuspend off: ' + ','.join(notes)) if notes else 'no 2bc5 device yet'


def clear_orbbec_device_lock() -> str:
    """Drop leftover Orbbec shm lock (often left by a previous sudo launch)."""
    notes: List[str] = []
    for path in ORBBEC_SHM_LOCKS:
        if not os.path.exists(path):
            continue
        try:
            os.unlink(path)
            notes.append(f'removed {path}')
        except PermissionError:
            notes.append(
                f'cannot remove {path} (owned by root). '
                f'Run: sudo rm -f {" ".join(ORBBEC_SHM_LOCKS)}')
        except OSError as exc:
            notes.append(f'{path}: {exc}')
    return '; '.join(notes)


# Gemini E 640x360 MJPG/YUYV/RGB: 5, 10, 15, 20, 25, 30 — no 3fps.
_GEMINI_E_FPS = (5, 10, 15, 20, 25, 30)


def _snap_fps(raw: str) -> str:
    try:
        want = int(float(raw))
    except (TypeError, ValueError):
        return '5'
    for fps in _GEMINI_E_FPS:
        if want <= fps:
            return str(fps)
    return '30'


def dabai_launch_args(
        *,
        color_width: Optional[str] = None,
        color_height: Optional[str] = None,
        color_fps: Optional[str] = None,
        depth_fps: Optional[str] = None) -> Dict[str, str]:
    args = dict(DABAI_LAUNCH_ARGS)
    if color_width:
        args['color_width'] = str(int(color_width))
    if color_height:
        args['color_height'] = str(int(color_height))
    if color_fps:
        fps = _snap_fps(color_fps)
        args['color_fps'] = fps
        args['depth_fps'] = _snap_fps(depth_fps) if depth_fps else fps
    elif depth_fps:
        args['depth_fps'] = _snap_fps(depth_fps)
    return args


def dabai_cli_args(**overrides) -> List[str]:
    args = dabai_launch_args(**overrides) if overrides else dict(DABAI_LAUNCH_ARGS)
    return [f'{key}:={value}' for key, value in args.items()]


def streams_off_cmd() -> List[str]:
    """Idle: stop color+depth after the USB device is actually up.

    Gemini E on USB2 can take 30–60s before toggle_* exists. Do not exit
    non-zero if the call times out — the camera node is still starting.
    """
    return [
        'python3', '-c',
        (
            'import subprocess,sys,time\n'
            't=time.time()+90\n'
            'while time.time()<t:\n'
            " r=subprocess.run(['ros2','service','list'],capture_output=True,text=True)\n"
            " if '/camera/toggle_color' in r.stdout: break\n"
            ' time.sleep(1.0)\n'
            'else: sys.exit(0)\n'
            'time.sleep(4)\n'
            "for s in ('/camera/toggle_depth','/camera/toggle_color'):\n"
            ' try:\n'
            "  subprocess.run(['ros2','service','call',s,'std_srvs/srv/SetBool',"
            "'{data: false}'],timeout=12,check=False)\n"
            ' except Exception:\n'
            '  pass\n'
        ),
    ]
