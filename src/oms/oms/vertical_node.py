#!/usr/bin/env python3
"""OMS vertical servo (same angles as mypackage oms_node Y/H).

Service /vertical_servo/set_servo, initparams.speed = degrees.
  up   =  50°   (Y)
  down = −40°   (H)  — home. Forced on node start.

  ros2 topic pub --once /oms/vertical/cmd std_msgs/msg/String \\
    '{data: "{\"mode\":\"up\"}"}'
"""

from __future__ import annotations

import json
import os
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from studica_control.srv import SetData


def _studio_log_path(name: str) -> Path:
    for root in (
            Path('/home/vmx/skill23_ws/studio_maps/_logs'),
            Path(os.path.expanduser('~/skill23_ws/studio_maps/_logs'))):
        try:
            root.mkdir(parents=True, exist_ok=True)
            return root / name
        except OSError:
            continue
    return Path('/tmp') / name


class OmsVerticalNode(Node):
    def __init__(self) -> None:
        super().__init__('oms_vertical')
        self.declare_parameter('service', '/vertical_servo/set_servo')
        self.declare_parameter('up_deg', 50.0)
        self.declare_parameter('down_deg', -40.0)
        self.declare_parameter('settle_sec', 0.8)
        self.declare_parameter('auto_home', True)

        self._up = float(self.get_parameter('up_deg').value)
        self._down = float(self.get_parameter('down_deg').value)
        self._settle = max(0.2, float(self.get_parameter('settle_sec').value))
        self._pose = 'unknown'
        self._angle = None
        self._homed = False
        self._state = 'boot'
        self._job_gen = 0
        self._cancel = threading.Event()
        self._log_path = _studio_log_path('oms.log')

        srv = str(self.get_parameter('service').value)
        self._cli = self.create_client(SetData, srv)
        self._result_pub = self.create_publisher(String, '/oms/vertical/result', 10)
        self._status_pub = self.create_publisher(String, '/oms/vertical/status', 10)
        self.create_subscription(String, '/oms/vertical/cmd', self._on_cmd, 10)
        self.create_subscription(String, '/oms/vertical/set_pose', self._on_set_pose, 10)
        self.create_timer(0.2, self._publish_status)
        self.get_logger().info(
            f'OMS vertical  {srv}  up={self._up:g}°  down(home)={self._down:g}°')
        if bool(self.get_parameter('auto_home').value):
            threading.Thread(target=self._boot_home, daemon=True).start()

    def _flog(self, level: str, msg: str) -> None:
        line = f'{time.strftime("%H:%M:%S")} [{level}] {msg}'
        log = self.get_logger()
        if level == 'error':
            log.error(f'[oms_vertical] {msg}')
        elif level == 'warn':
            log.warn(f'[oms_vertical] {msg}')
        else:
            log.info(f'[oms_vertical] {msg}')
        try:
            with open(self._log_path, 'a', encoding='utf-8') as handle:
                handle.write(line + '\n')
        except Exception:
            pass

    def _publish_status(self) -> None:
        payload = {
            'state': self._state,
            'homed': self._homed,
            'pose': self._pose,
            'angle_deg': self._angle,
            'up_deg': self._up,
            'down_deg': self._down,
        }
        try:
            self._status_pub.publish(String(data=json.dumps(payload)))
        except Exception:
            pass

    def _pub_result(self, cmd_id: str, ok: bool, message: str) -> None:
        payload = {
            'cmd_id': cmd_id,
            'success': bool(ok),
            'mode': 'vertical',
            'message': message,
            'pose': self._pose,
            'angle_deg': self._angle,
            'homed': self._homed,
        }
        try:
            self._result_pub.publish(String(data=json.dumps(payload)))
        except Exception:
            pass
        self._flog('info' if ok else 'warn', f'result ok={ok} {message}')

    def _wait_servo(self, timeout: float = 20.0) -> bool:
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < timeout:
            if self._cli.wait_for_service(timeout_sec=0.5):
                return True
        return self._cli.service_is_ready()

    def _set_angle(self, deg: float) -> bool:
        if not self._cli.service_is_ready():
            return False
        req = SetData.Request()
        req.initparams.speed = float(deg)
        fut = self._cli.call_async(req)
        t0 = time.time()
        while rclpy.ok() and not fut.done() and (time.time() - t0) < 8.0:
            time.sleep(0.02)
        if not fut.done():
            return False
        try:
            res = fut.result()
        except Exception:
            return False
        if bool(getattr(res, 'success', False)):
            return True
        msg = str(getattr(res, 'message', '') or '').lower()
        return 'already' in msg

    def _goto_pose(self, pose: str) -> bool:
        pose = 'down' if pose in ('home', 'down') else 'up'
        target = self._down if pose == 'down' else self._up
        self._state = pose
        ok = False
        for attempt in range(1, 6):
            ok = self._set_angle(target)
            if ok:
                break
            self._flog('warn', f'vertical {pose} attempt {attempt}/5 failed — retry')
            time.sleep(1.0)
        if ok:
            time.sleep(self._settle)
            self._pose = pose
            self._angle = target
            self._homed = pose == 'down'
            self._state = 'idle'
            self._flog('info', f'vertical {pose} → {target:g}°')
        else:
            self._state = 'failed'
            self._flog('error', f'vertical {pose} failed (is /vertical_servo/set_servo up?)')
        return ok

    def _boot_home(self) -> None:
        self._state = 'wait_servo'
        if not self._wait_servo():
            self._state = 'home_failed'
            self._flog('error', 'vertical servo service not ready — cannot home')
            self._pub_result('', False, 'boot home failed')
            return
        ok = self._goto_pose('down')
        self._pub_result('', ok, 'homed down' if ok else 'boot home failed')

    def _on_set_pose(self, msg: String) -> None:
        pose = str(msg.data or '').strip().lower()
        self._on_cmd(String(data=json.dumps({
            'cmd_id': f'set_{int(time.time() * 1000)}',
            'mode': pose or 'down',
        })))

    def _on_cmd(self, msg: String) -> None:
        try:
            cmd = json.loads(msg.data)
        except Exception as exc:
            self._pub_result('', False, f'bad JSON: {exc}')
            return
        if not isinstance(cmd, dict):
            self._pub_result('', False, 'command must be an object')
            return
        self._cancel.set()
        self._job_gen += 1
        gen = self._job_gen
        threading.Thread(
            target=self._run_cmd, args=(cmd, gen), daemon=True).start()

    def _run_cmd(self, cmd: dict, gen: int) -> None:
        time.sleep(0.02)
        self._cancel.clear()
        if gen != self._job_gen:
            return
        cmd_id = str(cmd.get('cmd_id') or '')
        mode = str(cmd.get('mode') or 'down').strip().lower()
        if mode in ('up', 'down', 'home'):
            ok = self._goto_pose(mode)
            label = 'up' if mode == 'up' else 'down'
            self._pub_result(cmd_id, ok, label if ok else f'{label} failed')
            return
        self._pub_result(cmd_id, False, f'unknown mode {mode}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OmsVerticalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
