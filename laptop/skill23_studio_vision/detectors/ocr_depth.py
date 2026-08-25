"""OCR: align on text box if possible, then depth approach."""

from __future__ import annotations

from typing import Any, Dict

from detectors import depth_util, ocr


def detect(frame, params: Dict[str, Any], depth=None) -> Dict[str, Any]:
    res = ocr.detect(frame, params)
    return depth_util.annotate(res, depth, frame.shape)
