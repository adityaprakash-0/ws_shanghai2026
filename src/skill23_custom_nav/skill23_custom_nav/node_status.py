"""Helpers for one-shot state logs and dashboard node status."""

from __future__ import annotations

import json
from typing import Any, Optional

from rclpy.node import Node
from std_msgs.msg import String


class NodeStatusPub:
    """Publish JSON status on /custom_nav/node_status; log only on change."""

    def __init__(self, node: Node, name: str) -> None:
        self._node = node
        self._name = name
        self._pub = node.create_publisher(String, '/custom_nav/node_status', 10)
        self._last_state: Optional[str] = None
        self._last_detail: Optional[str] = None

    def set(self, state: str, detail: str = '', *, log: bool = True, **extra: Any) -> None:
        state = str(state)
        detail = str(detail or '')
        state_changed = state != self._last_state
        detail_changed = detail != self._last_detail
        self._last_state = state
        self._last_detail = detail
        payload = {'node': self._name, 'state': state, 'detail': detail, **extra}
        msg = String()
        msg.data = json.dumps(payload)
        self._pub.publish(msg)
        if log and (state_changed or detail_changed):
            line = f'[{self._name}] {state}'
            if detail:
                line += f' — {detail}'
            self._node.get_logger().info(line)
