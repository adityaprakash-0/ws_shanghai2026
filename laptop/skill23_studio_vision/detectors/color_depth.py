"""Color blob: align, then depth approach (any colour or a named one)."""

from __future__ import annotations

from typing import Any, Dict

from detectors import color, depth_util


def detect(frame, params: Dict[str, Any], depth=None) -> Dict[str, Any]:
    res = color.detect(frame, params)
    return depth_util.annotate(res, depth, frame.shape)
