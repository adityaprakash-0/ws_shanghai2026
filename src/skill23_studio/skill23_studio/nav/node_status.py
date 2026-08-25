"""Helpers for one-shot state logs and dashboard node status."""

from __future__ import annotations

import json
import time
from typing import Any, Optional

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


_LATCH = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class NodeStatusPub:
    """Publish JSON status on /custom_nav/node_status; log only on change."""

    def __init__(self, node: Node, name: str) -> None:
        self._node = node
        self._name = name
        self._pub = node.create_publisher(String, '/custom_nav/node_status', _LATCH)
        self._last_state: Optional[str] = None
        self._last_detail: Optional[str] = None
        self._last_extra: dict = {}
        self._last_payload: Optional[dict] = None
        node.create_timer(1.0, self._heartbeat)

    def _publish(self, payload: dict) -> None:
        payload = {
            **payload,
            'heartbeat': time.monotonic(),
        }
        self._last_payload = payload
        msg = String()
        msg.data = json.dumps(payload)
        self._pub.publish(msg)

    def _heartbeat(self) -> None:
        if self._last_payload is not None:
            self._publish({
                'node': self._name,
                'state': self._last_state or 'starting',
                'detail': self._last_detail or '',
                **self._last_extra,
            })

    def set(self, state: str, detail: str = '', *, log: bool = True, **extra: Any) -> None:
        state = str(state)
        detail = str(detail or '')
        state_changed = state != self._last_state
        detail_changed = detail != self._last_detail
        self._last_state = state
        self._last_detail = detail
        self._last_extra = dict(extra)
        payload = {'node': self._name, 'state': state, 'detail': detail, **extra}
        self._publish(payload)
        if log and (state_changed or detail_changed):
            line = f'[{self._name}] {state}'
            if detail:
                line += f' — {detail}'
            self._node.get_logger().info(line)
