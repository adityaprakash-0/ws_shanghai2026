"""Barcode: align, then depth approach."""

from __future__ import annotations

from typing import Any, Dict

from detectors import barcode, depth_util


def detect(frame, params: Dict[str, Any], depth=None) -> Dict[str, Any]:
    res = barcode.detect(frame, params)
    return depth_util.annotate(res, depth, frame.shape)
