"""Hardware Testing Software — expected channels from hw_params + live scalars."""

from __future__ import annotations

import math
import os
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

try:
    import yaml
except ImportError:
    yaml = None  # type: ignore


def hw_params_path(robot: str) -> Path:
    safe = robot.strip().lower()
    candidates = [
        Path(os.path.expanduser(
            f'~/skill23_ws/src/skill23_studio/config/hw_params/hw_params_{safe}.yaml')),
        Path(os.path.expanduser(
            f'~/skill23_ws/install/skill23_studio/share/skill23_studio/config/'
            f'hw_params/hw_params_{safe}.yaml')),
    ]
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('skill23_studio'))
        candidates.insert(0, share / 'config' / 'hw_params' / f'hw_params_{safe}.yaml')
    except Exception:
        pass
    for p in candidates:
        if p.is_file():
            return p
    raise FileNotFoundError(f'hw_params_{safe}.yaml not found')


def load_hw_params(robot: str) -> Dict[str, Any]:
    path = hw_params_path(robot)
    if yaml is None:
        raise RuntimeError('PyYAML required to read hw_params')
    raw = yaml.safe_load(path.read_text()) or {}
    return (raw.get('control_server') or {}).get('ros__parameters') or {}


def build_channel_specs(robot: str, *, include_lidar: bool = True) -> List[Dict[str, Any]]:
    """Return expected telemetry channels for enabled devices (+ lidars)."""
    params = load_hw_params(robot)
    channels: List[Dict[str, Any]] = []

    # Battery always published by control_server.
    channels.append({
        'id': 'battery.voltage',
        'group': 'battery',
        'label': 'Battery V',
        'topic': '/battery_state',
        'msg': 'sensor_msgs/BatteryState',
        'unit': 'V',
        'enabled': True,
    })
    channels.append({
        'id': 'battery.percent',
        'group': 'battery',
        'label': 'Battery %',
        'topic': '/battery_state',
        'msg': 'sensor_msgs/BatteryState',
        'unit': '%',
        'enabled': True,
    })

    titan = params.get('titan') or {}
    if titan.get('enabled'):
        for titan_name in titan.get('sensors') or []:
            cfg = titan.get(titan_name) or {}
            lim = bool(cfg.get('limit_switches'))
            for mi in range(4):
                base = f'/{titan_name}/m_{mi}'
                channels.append({
                    'id': f'{titan_name}.m_{mi}.encoder',
                    'group': 'titan',
                    'label': f'{titan_name} m{mi} enc',
                    'topic': f'{base}/encoder',
                    'msg': 'std_msgs/Float64',
                    'unit': 'ticks',
                    'enabled': True,
                })
                channels.append({
                    'id': f'{titan_name}.m_{mi}.rpm',
                    'group': 'titan',
                    'label': f'{titan_name} m{mi} rpm',
                    'topic': f'{base}/rpm',
                    'msg': 'std_msgs/Float64',
                    'unit': 'rpm',
                    'enabled': True,
                })
                if lim:
                    channels.append({
                        'id': f'{titan_name}.m_{mi}.limit_fwd',
                        'group': 'titan',
                        'label': f'{titan_name} m{mi} lim+',
                        'topic': f'{base}/limit_fwd',
                        'msg': 'std_msgs/Bool',
                        'unit': '',
                        'enabled': True,
                    })
                    channels.append({
                        'id': f'{titan_name}.m_{mi}.limit_rev',
                        'group': 'titan',
                        'label': f'{titan_name} m{mi} lim−',
                        'topic': f'{base}/limit_rev',
                        'msg': 'std_msgs/Bool',
                        'unit': '',
                        'enabled': True,
                    })

    imu = params.get('imu') or {}
    if imu.get('enabled'):
        topic = str(imu.get('topic') or 'imu').lstrip('/')
        channels.append({
            'id': 'imu.yaw',
            'group': 'imu',
            'label': 'IMU yaw',
            'topic': f'/{topic}',
            'msg': 'sensor_msgs/Imu',
            'unit': 'deg',
            'enabled': True,
        })
        for axis in ('x', 'y', 'z'):
            channels.append({
                'id': f'gyro.{axis}',
                'group': 'imu',
                'label': f'gyro_{axis}',
                'topic': f'/gyro_{axis}',
                'msg': 'std_msgs/Float32',
                'unit': 'rad/s',
                'enabled': True,
            })

    servo = params.get('servo') or {}
    if servo.get('enabled'):
        for name in servo.get('sensors') or []:
            channels.append({
                'id': f'servo.{name}',
                'group': 'servo',
                'label': name,
                'topic': f'/{name}/state',
                'msg': 'std_msgs/Float64',
                'unit': '',
                'enabled': True,
            })

    us = params.get('ultrasonic') or {}
    if us.get('enabled'):
        for name in us.get('sensors') or []:
            channels.append({
                'id': f'us.{name}',
                'group': 'ultrasonic',
                'label': name,
                'topic': f'/{name}/range',
                'msg': 'sensor_msgs/Range',
                'unit': 'm',
                'enabled': True,
            })

    sharp = params.get('sharp') or {}
    if sharp.get('enabled'):
        for name in sharp.get('sensors') or []:
            channels.append({
                'id': f'sharp.{name}',
                'group': 'ir',
                'label': name,
                'topic': f'/{name}/range',
                'msg': 'sensor_msgs/Range',
                'unit': 'm',
                'enabled': True,
            })

    cobra = params.get('cobra') or {}
    if cobra.get('enabled'):
        for name in cobra.get('sensors') or []:
            for ch in range(4):
                channels.append({
                    'id': f'cobra.{name}.ch_{ch}',
                    'group': 'cobra',
                    'label': f'{name} ch{ch}',
                    'topic': f'/{name}/ch_{ch}',
                    'msg': 'std_msgs/Float32',
                    'unit': 'V',
                    'enabled': True,
                })

    dio = params.get('dio') or {}
    if dio.get('enabled'):
        for name in dio.get('sensors') or []:
            channels.append({
                'id': f'dio.{name}',
                'group': 'dio',
                'label': name,
                'topic': f'/{name}/state',
                'msg': 'std_msgs/Bool',
                'unit': '',
                'enabled': True,
            })

    enc = params.get('encoder') or {}
    if enc.get('enabled'):
        for name in enc.get('sensors') or []:
            cfg = enc.get(name) or {}
            topic = str(cfg.get('topic') or name).lstrip('/')
            channels.append({
                'id': f'encoder.{name}',
                'group': 'encoder',
                'label': name,
                'topic': f'/{topic}',
                'msg': 'std_msgs/Float64',
                'unit': '',
                'enabled': True,
            })

    duty = params.get('duty_cycle') or {}
    if duty.get('enabled'):
        for name in duty.get('sensors') or []:
            cfg = duty.get(name) or {}
            topic = str(cfg.get('topic') or name).lstrip('/')
            channels.append({
                'id': f'duty.{name}',
                'group': 'encoder',
                'label': name,
                'topic': f'/{topic}',
                'msg': 'std_msgs/Float64',
                'unit': '',
                'enabled': True,
            })

    lt = params.get('light_tower') or {}
    if lt.get('enabled'):
        name = str(lt.get('name') or 'light_tower')
        channels.append({
            'id': 'light_tower.state',
            'group': 'light_tower',
            'label': 'Light tower',
            'topic': f'/{name}/state',
            'msg': 'std_msgs/String',
            'unit': '',
            'enabled': True,
        })

    if include_lidar:
        for topic, label in (
            ('/scan/front', 'Lidar front'),
            ('/scan/back', 'Lidar back'),
            ('/scan', 'Lidar merged'),
        ):
            channels.append({
                'id': f'lidar.{label}',
                'group': 'lidar',
                'label': label,
                'topic': topic,
                'msg': 'sensor_msgs/LaserScan',
                'unit': 'm',
                'enabled': True,
            })

    return channels


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def laser_min_range(ranges, range_min: float, range_max: float) -> Optional[float]:
    best = None
    step = max(1, len(ranges) // 360)
    for i in range(0, len(ranges), step):
        r = float(ranges[i])
        if math.isfinite(r) and range_min < r < range_max:
            if best is None or r < best:
                best = r
    return best


class HwLiveStore:
    """Thread-safe last-value store for HW test channels."""

    def __init__(self) -> None:
        self._lock = __import__('threading').Lock()
        self._values: Dict[str, Dict[str, Any]] = {}
        self._counts: Dict[str, int] = {}
        self._hz_t0 = time.time()
        self._hz_counts: Dict[str, int] = {}
        self._hz: Dict[str, float] = {}
        self.robot = ''
        self.specs: List[Dict[str, Any]] = []
        self.active = False

    def reset(self, robot: str, specs: List[Dict[str, Any]]) -> None:
        with self._lock:
            self.robot = robot
            self.specs = list(specs)
            self._values = {}
            self._counts = {}
            self._hz_counts = {}
            self._hz = {}
            self._hz_t0 = time.time()
            self.active = True

    def clear(self) -> None:
        with self._lock:
            self.active = False
            self._values = {}
            self.specs = []

    def set_value(self, channel_id: str, value: Any, topic: str = '') -> None:
        now = time.time()
        with self._lock:
            self._values[channel_id] = {
                'value': value,
                'topic': topic,
                'stamp': now,
            }
            self._counts[channel_id] = self._counts.get(channel_id, 0) + 1
            self._hz_counts[channel_id] = self._hz_counts.get(channel_id, 0) + 1
            if now - self._hz_t0 >= 1.0:
                dt = max(1e-3, now - self._hz_t0)
                for cid, n in self._hz_counts.items():
                    self._hz[cid] = round(n / dt, 1)
                self._hz_counts = {}
                self._hz_t0 = now

    def snapshot(self, *, only_live: bool = True, stale_s: float = 2.5) -> Dict[str, Any]:
        now = time.time()
        with self._lock:
            groups: Dict[str, List[Dict[str, Any]]] = {}
            live_n = 0
            for spec in self.specs:
                cid = spec['id']
                entry = self._values.get(cid)
                age = None
                value = None
                hz = self._hz.get(cid, 0.0)
                if entry:
                    age = round(now - float(entry['stamp']), 2)
                    value = entry['value']
                publishing = entry is not None and age is not None and age <= stale_s
                if only_live and not publishing:
                    continue
                if publishing:
                    live_n += 1
                row = {
                    'id': cid,
                    'group': spec['group'],
                    'label': spec['label'],
                    'topic': spec['topic'],
                    'unit': spec.get('unit') or '',
                    'value': value,
                    'age_s': age,
                    'hz': hz if publishing else 0.0,
                    'live': publishing,
                }
                groups.setdefault(spec['group'], []).append(row)
            return {
                'ok': True,
                'active': self.active,
                'robot': self.robot,
                'live_channels': live_n,
                'groups': groups,
                'ts': now,
            }
