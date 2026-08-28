#!/usr/bin/env python3
"""Launch / stop ROS processes for Skill23 Studio.

HW access (Titan / VMX):

  * Outer ``ros2 launch`` (slam / localize / nav) runs directly and inherits
    the Studio parent process user, privileges, and environment.
  * Inside that launch, ``manual_composition`` is ``sudo -E <binary> --ros-args``
    (same as slam_tele). Do not put ``env PATH=…`` in front of the binary —
    that crashes control_server with InvalidParameterValueException.

Why leftovers happen
--------------------
Deploy starts ``ros2 launch skill23_studio studio_nav.launch.py`` in a **new
session** (``start_new_session=True``). That launch then starts each node in
its *own* process group (and ``sudo manual_composition`` is yet another tree).

``stop()`` used to ``killpg`` only the launch PID, then ``wait()``. ROS 2 launch
often exits immediately; its node children are reparented to PID 1 and keep
running. Ctrl+C on ``studio.launch.py`` only signals studio_bridge — if
cleanup does not reap **by name**, grip/ekf/cn_*/lidars stay forever.

This module always: signal the launch tree, then name-reap survivors.
"""

from __future__ import annotations

import os
import signal
import subprocess
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Set


def _log_dir() -> Path:
    d = Path(os.path.expanduser('~/skill23_ws/studio_maps/_logs'))
    d.mkdir(parents=True, exist_ok=True)
    return d


def _stop_script_path() -> Optional[str]:
    p = Path(__file__).resolve().parent.parent / 'scripts' / 'stop_studio_stack.sh'
    return str(p) if p.is_file() else None


def _run_stop_script() -> None:
    script = _stop_script_path()
    if not script:
        return
    try:
        subprocess.run(
            ['bash', script],
            timeout=20,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except Exception:
        pass


@dataclass
class ManagedProcess:
    name: str
    popen: subprocess.Popen
    cmd: List[str] = field(default_factory=list)
    started_at: float = field(default_factory=time.time)
    log_path: Optional[str] = None


# Match cmdline fragments of leftover studio children (not studio.launch itself).
_NAV_ORPHANS = (
    'studio_nav_camera.launch.py',
    'studio_nav2.launch.py',
    'test_detect_align.launch.py',
    'test_wp_align.launch.py',
    'test_oms.launch.py',
    'studio_nav.launch.py',
    'studio_localize.launch.py',
    'slam_tele_grip.py',
    'slam_tele_allterrain.py',
    'slam_tele_mecanum.py',
    'slam_tele_xdrive.py',
    'lib/skill23_studio/cn_',
    'lib/skill23_studio/studio_align',
    'lib/skill23_studio/studio_detect_align',
    'lib/oms/oms_rotation',
    'lib/oms/oms_vertical',
    'cn_obstacle_detector',
    'cn_collision_checker',
    'cn_planner',
    'cn_path_smoother',
    'cn_controller',
    'cn_goal_checker',
    'cn_recovery',
    'cn_mission',
    'cn_world_model',
    'cn_bridge',
    'studio_camera_gate',
    'ydlidar_ros2_driver_node',
    'lidar_merger_node',
    'robot_localization/ekf_node',
    '__node:=studio_joy_node',
    'skill23_amcl',
    'lib/skill23_amcl/amcl',
    '__node:=amcl',
    '__node:=map_server',
    'lib/nav2_amcl/amcl',
    'lib/nav2_map_server/map_server',
    'lifecycle_manager_localization',
    'lib/grip/grip_node',
    'allterrain_node',
    'mecanum_node',
    'xdrive_node',
    'manual_composition',
    '__node:=control_server',
    '__node:=robot_state_publisher',
    '__node:=planner_server',
    '__node:=controller_server',
    '__node:=bt_navigator',
    '__node:=behavior_server',
    '__node:=smoother_server',
    '__node:=waypoint_follower',
    '__node:=velocity_smoother',
    '__node:=nav2_container',
    'scan_filtered_relay',
)
_CAM_ORPHANS = (
    'dabai.launch.py',
    'gemini_e.launch.py',
    'launch_gemini_e.sh',
    '__node:=camera_container',
    'ob_camera_node',
)
_KEEP_CMDS = (
    'studio.launch.py',
    'studio_bridge',
    'studio_stop_stack',
    'stop_studio_stack',
)


def _ppid(pid: int) -> Optional[int]:
    try:
        for line in Path(f'/proc/{pid}/status').read_text().splitlines():
            if line.startswith('PPid:'):
                return int(line.split()[1])
    except Exception:
        return None
    return None


def _children(pid: int) -> List[int]:
    kids: List[int] = []
    proc = Path('/proc')
    try:
        for entry in proc.iterdir():
            if not entry.name.isdigit():
                continue
            child = int(entry.name)
            if _ppid(child) == pid:
                kids.append(child)
    except Exception:
        pass
    return kids


def _descendants(pid: int) -> List[int]:
    found: List[int] = []
    stack = [pid]
    seen = {pid}
    while stack:
        cur = stack.pop()
        for child in _children(cur):
            if child not in seen:
                seen.add(child)
                found.append(child)
                stack.append(child)
    return found


def _signal_pids(pids: Iterable[int], sig: int) -> None:
    for pid in pids:
        try:
            os.kill(pid, sig)
        except PermissionError:
            try:
                subprocess.call(
                    ['sudo', '-n', 'kill', f'-{int(sig)}', str(pid)],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception:
                pass
        except (ProcessLookupError, OSError):
            pass


def _pkill_patterns(patterns: Iterable[str]) -> None:
    """Last-resort kill for sudo-launched orphans (pkill + sudo pkill).

    Patterns must be specific executable/launch fragments — NEVER bare package
    names like ``orbbec_camera`` (those appear inside AMENT_PREFIX_PATH on the
    Studio ``sudo -E env …`` command line and would kill Studio itself).

    Scan /proc once and only invoke pkill for patterns that actually match;
    otherwise selecting a map spends many seconds on empty pkill sweeps.
    """
    cmdlines = [cmd for _pid, cmd in _list_cmdlines()]
    for pat in patterns:
        if not any(pat in cmd for cmd in cmdlines):
            continue
        for cmd in (
            ['pkill', '-9', '-f', '--', pat],
            ['sudo', '-n', 'pkill', '-9', '-f', '--', pat],
        ):
            try:
                subprocess.call(
                    cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception:
                pass


# Safe for pkill -f (will not match Studio's sudo env AMENT_PREFIX_PATH line).
_PKILL_SAFE = (
    'studio_nav_camera.launch.py',
    'studio_nav2.launch.py',
    'test_detect_align.launch.py',
    'test_wp_align.launch.py',
    'test_oms.launch.py',
    'studio_nav.launch.py',
    'studio_localize.launch.py',
    'slam_tele_grip.py',
    'slam_tele_allterrain.py',
    'slam_tele_mecanum.py',
    'slam_tele_xdrive.py',
    'manual_composition',
    '__node:=control_server',
    'lib/grip/grip_node',
    'allterrain_node',
    'mecanum_node',
    'xdrive_node',
    'lib/skill23_studio/cn_',
    'lib/skill23_studio/studio_align',
    'lib/skill23_studio/studio_detect_align',
    'lib/oms/oms_rotation',
    'lib/oms/oms_vertical',
    'cn_obstacle_detector',
    'cn_collision_checker',
    'cn_planner',
    'cn_path_smoother',
    'cn_controller',
    'cn_goal_checker',
    'cn_recovery',
    'cn_mission',
    'cn_world_model',
    'studio_camera_gate',
    'ydlidar_ros2_driver_node',
    'lidar_merger_node',
    '__node:=studio_joy_node',
    '__node:=amcl',
    '__node:=map_server',
    'lib/skill23_amcl/amcl',
    'lib/nav2_amcl/amcl',
    'lib/nav2_map_server/map_server',
    'lifecycle_manager_localization',
    'gemini_e.launch.py',
    'launch_gemini_e.sh',
    '__node:=camera_container',
    'ob_camera_node',
)


def _list_cmdlines() -> List[tuple]:
    rows: List[tuple] = []
    try:
        out = subprocess.check_output(
            ['ps', '-eo', 'pid=,args='], text=True, timeout=3)
    except Exception:
        return rows
    for line in out.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            pid_s, cmd = line.split(None, 1)
            rows.append((int(pid_s), cmd))
        except ValueError:
            continue
    return rows


def reap_studio_orphans(
    *,
    nav: bool = True,
    camera: bool = True,
    keep_pids: Optional[Set[int]] = None,
    force_pkill: bool = False,
) -> int:
    """Kill leftover studio nav/camera processes (orphans after launch died)."""
    needles: List[str] = []
    if nav:
        needles.extend(_NAV_ORPHANS)
    if camera:
        needles.extend(_CAM_ORPHANS)
    if not needles:
        return 0
    keep = set(keep_pids or ())
    keep.add(os.getpid())
    keep.add(os.getppid())
    pids: List[int] = []
    for pid, cmd in _list_cmdlines():
        if pid in keep:
            continue
        if any(k in cmd for k in _KEEP_CMDS):
            continue
        # Never touch the Studio launcher itself (sudo env embeds AMENT paths).
        if 'studio.launch.py' in cmd:
            continue
        if any(n in cmd for n in needles):
            pids.append(pid)
    if pids:
        _signal_pids(pids, signal.SIGTERM)
        time.sleep(0.25)
        _signal_pids(pids, signal.SIGKILL)
    # Blind pkill only on explicit shutdown — never on bridge startup reap
    # (pkill -f orbbec_camera matched AMENT_PREFIX_PATH and killed Studio).
    if force_pkill:
        safe = []
        if nav:
            safe.extend([p for p in _PKILL_SAFE if 'gemini' not in p and 'camera' not in p and 'ob_camera' not in p])
        if camera:
            safe.extend([
                'dabai.launch.py',
                'gemini_e.launch.py', 'launch_gemini_e.sh',
                '__node:=camera_container', 'ob_camera_node',
            ])
        _pkill_patterns(safe)
    return len(pids)


def main_stop() -> None:
    n = reap_studio_orphans(force_pkill=True)
    print(f'studio_stop_stack: reaped {n} process(es)')


def _cmdline_has(needle: str) -> bool:
    for _pid, cmd in _list_cmdlines():
        if needle in cmd and not any(k in cmd for k in _KEEP_CMDS):
            return True
    return False


class ProcessManager:
    def __init__(self) -> None:
        self._procs: Dict[str, ManagedProcess] = {}
        self._stack_lock = threading.Lock()

    def running(self, name: str) -> bool:
        p = self._procs.get(name)
        if p is None:
            return False
        return p.popen.poll() is None

    def list_running(self) -> List[str]:
        return [n for n in self._procs if self.running(n)]

    def stacks_alive(self) -> Dict[str, bool]:
        """Managed launch *or* leftover cmdline still present."""
        return {
            'nav': (
                self.running('nav')
                or _cmdline_has('studio_nav.launch.py')
                or _cmdline_has('studio_nav2.launch.py')
                or _cmdline_has('studio_nav_camera.launch.py')),
            'localize': (
                self.running('localize')
                or _cmdline_has('studio_localize.launch.py')),
            'slam': (
                self.running('slam')
                or _cmdline_has('slam_tele_grip.py')
                or _cmdline_has('slam_tele_allterrain.py')
                or _cmdline_has('slam_tele_mecanum.py')
                or _cmdline_has('slam_tele_xdrive.py')),
            'camera': (
                self.running('camera')
                or _cmdline_has('dabai.launch.py')
                or _cmdline_has('gemini_e.launch.py')
                or _cmdline_has('__node:=camera_container')),
        }

    def nav2_component_status(self) -> Dict[str, Dict[str, Any]]:
        """Process-level health for Nav2 stack status panel."""
        stack_up = (
            self.running('nav')
            or _cmdline_has('studio_nav2.launch.py')
            or _cmdline_has('__node:=nav2_container')
            or _cmdline_has('component_container_isolated'))
        checks = (
            ('amcl', ('__node:=amcl', 'skill23_amcl')),
            ('map_server', ('__node:=map_server', 'nav2_map_server')),
            ('planner', ('__node:=planner_server',)),
            ('controller', ('__node:=controller_server',)),
            ('bt_navigator', ('__node:=bt_navigator',)),
            ('behavior', ('__node:=behavior_server',)),
            ('smoother', ('__node:=smoother_server',)),
            ('ekf', ('robot_localization/ekf_node', 'ekf_node')),
            ('lidar', ('ydlidar_ros2_driver_node', 'lidar_merger_node')),
        )
        composable = {
            'amcl', 'map_server', 'planner', 'controller',
            'bt_navigator', 'behavior', 'smoother',
        }
        out: Dict[str, Dict[str, Any]] = {}
        for key, needles in checks:
            alive = any(_cmdline_has(n) for n in needles)
            if key in composable:
                alive = alive or stack_up
            out[key] = {
                'node': key,
                'state': 'ready' if alive else 'stopped',
                'detail': 'process up' if alive else 'not running',
            }
        return out

    def require_stacks_idle(self, *, wanting: str) -> None:
        """Refuse to start if another HW stack (or the same one) is already up."""
        alive = self.stacks_alive()
        blockers = [n for n in ('nav', 'localize', 'slam') if alive.get(n)]
        if not blockers:
            return
        names = ', '.join(blockers)
        raise RuntimeError(
            f'Cannot start {wanting}: already running ({names}). '
            f'Stop that stack first, then start again.')

    def _keep_pids(self) -> Set[int]:
        keep = {os.getpid(), os.getppid()}
        for mp in self._procs.values():
            if mp.popen.poll() is None:
                keep.add(int(mp.popen.pid))
        return keep

    def _wait_stack_idle(self, name: str, timeout: float = 12.0) -> None:
        """Block until no managed launch or cmdline match for this stack."""
        if name not in ('nav', 'localize', 'slam'):
            return
        deadline = time.time() + timeout
        while time.time() < deadline:
            if not self.stacks_alive()[name]:
                return
            reap_studio_orphans(
                nav=True, camera=False, keep_pids=self._keep_pids(),
                force_pkill=True)
            time.sleep(0.25)
        alive = self.stacks_alive()
        if alive.get(name):
            raise RuntimeError(
                f'{name} stack still running after {timeout:.0f}s '
                f'(leftover nodes). Run stop_studio_stack.sh, then retry.')

    def _stop_other_hw_stacks(self, *except_names: str) -> None:
        skip = set(except_names)
        for name in ('nav', 'localize', 'slam'):
            if name in skip:
                continue
            if self.stacks_alive()[name]:
                self.stop(name)
                self._wait_stack_idle(name)

    def _launch(
        self,
        name: str,
        args: List[str],
        *,
        cwd: Optional[str] = None,
        raw_cmd: Optional[List[str]] = None,
    ) -> ManagedProcess:
        if raw_cmd:
            cmd = list(raw_cmd)
        else:
            cmd = ['ros2'] + args

        log_path = _log_dir() / f'{name}.log'
        stamp = time.strftime('%Y-%m-%d %H:%M:%S')
        with open(log_path, 'a', encoding='utf-8') as log_f:
            log_f.write(f'\n# cmd [{stamp}]: {" ".join(cmd)}\n')
            log_f.flush()

        log_f = open(log_path, 'a', encoding='utf-8')
        popen = subprocess.Popen(
            cmd,
            cwd=cwd or os.path.expanduser('~'),
            stdout=log_f,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        mp = ManagedProcess(
            name=name, popen=popen, cmd=cmd, log_path=str(log_path))
        self._procs[name] = mp
        return mp

    def start(
        self,
        name: str,
        args: List[str],
        *,
        cwd: Optional[str] = None,
        raw_cmd: Optional[List[str]] = None,
    ) -> ManagedProcess:
        with self._stack_lock:
            self.stop(name)
            if name in ('nav', 'localize', 'slam'):
                self._wait_stack_idle(name)
            return self._launch(
                name, args, cwd=cwd, raw_cmd=raw_cmd)

    def stop(self, name: str, timeout: float = 1.2) -> None:
        """Stop a managed launch *and* every descendant node.

        If the launch PID is already dead, still name-reap that stack so
        orphaned cn_*/lidars/ekf/camera cannot survive.
        """
        p = self._procs.pop(name, None)
        tree: List[int] = []
        if p is not None and p.popen.poll() is None:
            root = int(p.popen.pid)
            tree = [root] + _descendants(root)
            _signal_pids(tree, signal.SIGINT)
            try:
                p.popen.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                pass
            # Launch may have died and reparented children — refresh + KILL.
            tree = list({*tree, root, *_descendants(root)})
            _signal_pids(tree, signal.SIGKILL)
            try:
                p.popen.wait(timeout=0.4)
            except Exception:
                pass
        if name in ('nav', 'localize', 'slam'):
            _run_stop_script()
            reap_studio_orphans(
                nav=True, camera=False, keep_pids=self._keep_pids(),
                force_pkill=True)
        elif name == 'camera':
            reap_studio_orphans(
                nav=False, camera=True, keep_pids=self._keep_pids(),
                force_pkill=True)

    def stop_all(self, timeout: float = 0.6) -> None:
        """Fast shutdown for Ctrl+C — signal every tree, then name-reap.

        When nothing is managed and no leftover studio stacks are alive this
        returns immediately. Map selection used to take many seconds here
        because every offline session call ran dozens of empty pkill sweeps.
        """
        trees: List[int] = []
        pops: List[subprocess.Popen] = []
        for name in list(self._procs.keys()):
            p = self._procs.pop(name, None)
            if p is None or p.popen.poll() is not None:
                continue
            root = int(p.popen.pid)
            trees.extend([root] + _descendants(root))
            pops.append(p.popen)
        if trees:
            _signal_pids(trees, signal.SIGINT)
            deadline = time.time() + timeout
            for popen in pops:
                left = max(0.05, deadline - time.time())
                try:
                    popen.wait(timeout=left)
                except subprocess.TimeoutExpired:
                    pass
            _signal_pids(trees, signal.SIGKILL)
            for popen in pops:
                try:
                    popen.wait(timeout=0.2)
                except Exception:
                    pass
        alive = self.stacks_alive()
        if not trees and not any(alive.values()):
            return
        # One cleanup path only — the bash stop script ends by calling
        # main_stop()/force_pkill, so running both doubles the cost.
        reap_studio_orphans(keep_pids=self._keep_pids(), force_pkill=True)

    def reap_orphans(self, *, nav: bool = True, camera: bool = True) -> int:
        return reap_studio_orphans(
            nav=nav, camera=camera, keep_pids=self._keep_pids())

    def start_slam(self, robot: str, slam_params_file: Optional[str] = None) -> ManagedProcess:
        if robot not in ('allterrain', 'grip', 'mecanum', 'xdrive'):
            raise ValueError(f'unknown robot {robot}')
        args = ['launch', 'studica_control', f'slam_tele_{robot}.py']
        if slam_params_file:
            args.append(f'slam_params_file:={slam_params_file}')
        with self._stack_lock:
            self._stop_other_hw_stacks()
            self.stop('slam')
            self._wait_stack_idle('slam')
            return self._launch('slam', args)

    def stop_slam(self) -> None:
        with self._stack_lock:
            self.stop('slam')
            self._wait_stack_idle('slam')

    def start_localize(self, robot: str, map_yaml: str) -> ManagedProcess:
        if robot not in ('allterrain', 'grip', 'mecanum', 'xdrive'):
            raise ValueError(f'unknown robot {robot}')
        if not map_yaml or not os.path.isfile(map_yaml):
            raise FileNotFoundError(f'map not found: {map_yaml}')
        with self._stack_lock:
            if self.running('localize') or _cmdline_has('studio_localize.launch.py'):
                existing = self._procs.get('localize')
                if existing is not None and existing.popen.poll() is None:
                    return existing
                raise RuntimeError('localize already running')
            self._stop_other_hw_stacks()
            self.stop('localize')
            self._wait_stack_idle('localize')
            return self._launch(
                'localize',
                [
                    'launch', 'skill23_studio', 'studio_localize.launch.py',
                    f'robot:={robot}',
                    f'map:={map_yaml}',
                    'enable_lidar:=true',
                ],
            )

    def stop_localize(self) -> None:
        with self._stack_lock:
            self.stop('localize')
            self._wait_stack_idle('localize')

    def start_nav(
            self, robot: str, map_yaml: str, *,
            with_camera: bool = False,
            with_align: Optional[bool] = None,
            stack: str = 'custom') -> ManagedProcess:
        if not map_yaml or not os.path.isfile(map_yaml):
            raise FileNotFoundError(f'map not found: {map_yaml}')
        stack = 'nav2' if str(stack).lower() == 'nav2' else 'custom'
        if with_align is None:
            with_align = not with_camera
        with self._stack_lock:
            # Localize/slam share grip/ekf/lidar — stop them first so orphans
            # cannot survive beside a new nav stack (duplicate cn_* / grip_node).
            self._stop_other_hw_stacks()
            self.stop('nav')
            self._wait_stack_idle('nav')
            if stack == 'nav2':
                launch_file = 'studio_nav2.launch.py'
                args = [
                    'launch', 'skill23_studio', launch_file,
                    f'robot:={robot}',
                    f'map:={map_yaml}',
                    f'with_camera:={"true" if with_camera else "false"}',
                    f'with_align:={"true" if with_align else "false"}',
                ]
            else:
                launch_file = (
                    'studio_nav_camera.launch.py'
                    if with_camera else 'studio_nav.launch.py')
                args = [
                    'launch', 'skill23_studio', launch_file,
                    f'robot:={robot}',
                    f'map:={map_yaml}',
                    'enable_bridge:=false',
                    'http_port:=8084',
                ]
            return self._launch('nav', args)

    def stop_nav(self) -> None:
        with self._stack_lock:
            self.stop('nav')
            self._wait_stack_idle('nav')

    def start_camera(self, *, enable_depth: bool = False) -> ManagedProcess:
        """Orbbec Gemini E — node up with toggle_color / toggle_depth.

        enable_color/depth stay true so the driver creates the toggle
        services; the bridge then stops the streams until Detect+Align.
        """
        from skill23_studio.orbbec_dabai import (
            clear_orbbec_device_lock, dabai_cli_args, prefer_workspace_orbbec,
            wake_orbbec_usb)
        self.stop_camera()
        clear_orbbec_device_lock()
        wake_orbbec_usb()
        prefer_workspace_orbbec()
        extra = dabai_cli_args()
        if enable_depth:
            extra = [
                ('enable_depth:=true' if a.startswith('enable_depth:=') else a)
                for a in extra
            ]
        return self.start(
            'camera',
            ['launch', 'orbbec_camera', 'gemini_e.launch.py'] + extra,
        )

    def stop_camera(self) -> None:
        self.stop('camera')

    def save_map(self, out_prefix: str) -> subprocess.CompletedProcess:
        """Save /map to out_prefix.yaml + .pgm via map_saver_cli (same user as bridge)."""
        cmd = [
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', out_prefix,
        ]
        return subprocess.run(
            cmd, capture_output=True, text=True, timeout=60,
        )
