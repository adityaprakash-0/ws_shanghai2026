"""Shared idle/active gate for the custom nav nodes.

The Pi cannot afford every nav node running flat out while the robot is
parked. Nodes keep publishing when idle — the startup gate and the dashboard
both need live data — but at a fraction of the navigating rate.

A node is considered active when the mission manager has enabled the
controller or reports a state that involves motion. Nodes also stay active for
a warm-up window after launch so the startup gate sees data immediately.
"""

from __future__ import annotations

import json
import time
from typing import Optional

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

BUSY_STATES = ('running', 'wait_goal', 'recovering')

# Keep running at full rate briefly after the last motion so a node is never
# throttled in the middle of a stop/replan handshake.
_LINGER_SEC = 3.0


class ActivityGate:
    def __init__(
        self,
        node: Node,
        active_hz: float,
        idle_hz: float = 2.0,
        warmup_sec: float = 20.0,
    ) -> None:
        self._node = node
        self._active_period = 1.0 / max(0.1, active_hz)
        self._idle_period = 1.0 / max(0.1, idle_hz)
        self._until = time.monotonic() + max(0.0, warmup_sec)
        self._enabled = False
        self._last = 0.0

        node.create_subscription(
            Bool, '/custom_nav/controller/enable', self._on_enable, LATCHED)
        node.create_subscription(
            String, '/custom_nav/mission/status', self._on_mission, 10)

    def _on_enable(self, msg: Bool) -> None:
        self._enabled = bool(msg.data)
        if self._enabled:
            self._touch()

    def _on_mission(self, msg: String) -> None:
        try:
            state = str(json.loads(msg.data).get('state', '')).lower()
        except Exception:
            return
        if state in BUSY_STATES:
            self._touch()

    def _touch(self) -> None:
        self._until = time.monotonic() + _LINGER_SEC

    @property
    def active(self) -> bool:
        return self._enabled or time.monotonic() < self._until

    @property
    def period(self) -> float:
        return self._active_period if self.active else self._idle_period

    def due(self, now: Optional[float] = None) -> bool:
        """True when enough time has passed for the current activity level."""
        stamp = time.monotonic() if now is None else now
        if stamp - self._last < self.period:
            return False
        self._last = stamp
        return True
