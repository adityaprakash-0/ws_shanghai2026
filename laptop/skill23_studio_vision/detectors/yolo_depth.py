"""YOLO: align on class, then depth approach."""

from __future__ import annotations

from typing import Any, Dict

from detectors import depth_util, yolo_det


def detect(frame, params: Dict[str, Any], depth=None) -> Dict[str, Any]:
    res = yolo_det.detect(frame, params)
    return depth_util.annotate(res, depth, frame.shape)
