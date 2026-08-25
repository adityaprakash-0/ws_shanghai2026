"""QR: align on code, then depth approach."""

from __future__ import annotations

from typing import Any, Dict

from detectors import depth_util, qr


def detect(frame, params: Dict[str, Any], depth=None) -> Dict[str, Any]:
    res = qr.detect(frame, params)
    return depth_util.annotate(res, depth, frame.shape)
