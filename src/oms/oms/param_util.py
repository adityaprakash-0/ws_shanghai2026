"""Helpers so OMS nodes accept yaml ints (90) or floats (90.0) for doubles."""

from __future__ import annotations

from typing import Any, Iterable, Sequence, Union

from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node

_DYN = ParameterDescriptor(dynamic_typing=True)


def declare_float(node: Node, name: str, default: float) -> float:
    """Declare a float param that still loads if yaml wrote an integer."""
    node.declare_parameter(name, float(default), _DYN)
    return float(node.get_parameter(name).value)


def declare_floats(node: Node, items: Sequence[tuple[str, float]]) -> dict[str, float]:
    out: dict[str, float] = {}
    for name, default in items:
        out[name] = declare_float(node, name, default)
    return out


def declare_int(node: Node, name: str, default: int) -> int:
    node.declare_parameter(name, int(default))
    return int(node.get_parameter(name).value)


def declare_bool(node: Node, name: str, default: bool) -> bool:
    node.declare_parameter(name, bool(default))
    return bool(node.get_parameter(name).value)


def declare_str(node: Node, name: str, default: str) -> str:
    node.declare_parameter(name, str(default))
    return str(node.get_parameter(name).value)


def as_float(value: Any, fallback: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(fallback)
