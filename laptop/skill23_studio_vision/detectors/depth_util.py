"""Shared depth sampling + standoff math.

Dashboard field ``approach_cm``: if depth is 20 cm and you want to stop at 15 cm,
enter 5. Target = measured_depth − 5 cm.

Optional ``stop_at_cm``: absolute remaining distance (e.g. 15). Wins if > 0.
"""

from __future__ import annotations

from typing import Any, Dict, Optional, Tuple

import numpy as np


def add_offset(res: Dict[str, Any], frame_shape) -> Dict[str, Any]:
    h, w = frame_shape[:2]
    if 'cx' in res and w > 1:
        res['offset_x'] = (float(res['cx']) - w / 2.0) / (w / 2.0)
        res['offset_y'] = (float(res.get('cy', h / 2.0)) - h / 2.0) / max(1.0, h / 2.0)
    return res


def depth_to_meters(depth) -> Optional[np.ndarray]:
    if depth is None:
        return None
    arr = np.asarray(depth)
    if arr.ndim != 2 or arr.size == 0:
        return None
    if np.issubdtype(arr.dtype, np.floating):
        out = arr.astype(np.float32)
        out[out > 20.0] = 0.0  # sometimes published in mm as float
        if np.nanmax(out) > 20.0:
            out = out / 1000.0
        return out
    # 16-bit millimetres (Orbbec)
    mm = arr.astype(np.float32)
    mm[mm <= 0] = 0.0
    return mm / 1000.0


def sample_m(depth, cx: float, cy: float, window: int = 9) -> Optional[float]:
    meters = depth_to_meters(depth)
    if meters is None:
        return None
    h, w = meters.shape
    x = int(round(cx * w / max(1.0, w)))  # already in depth px if registered
    y = int(round(cy))
    # If caller passed color pixels and sizes differ, caller should scale.
    x = int(np.clip(round(cx), 0, w - 1))
    y = int(np.clip(round(cy), 0, h - 1))
    r = max(1, window // 2)
    patch = meters[max(0, y - r):y + r + 1, max(0, x - r):x + r + 1]
    valid = patch[np.isfinite(patch) & (patch > 0.05) & (patch < 8.0)]
    if valid.size == 0:
        return None
    return float(np.median(valid))


def scale_xy(cx: float, cy: float, color_shape, depth_shape) -> Tuple[float, float]:
    ch, cw = color_shape[:2]
    dh, dw = depth_shape[:2]
    if cw <= 0 or ch <= 0:
        return cx, cy
    return cx * dw / cw, cy * dh / ch


def annotate(res: Dict[str, Any], depth, frame_shape) -> Dict[str, Any]:
    add_offset(res, frame_shape)
    if not res.get('ok') or depth is None or 'cx' not in res:
        return res
    meters = depth_to_meters(depth)
    if meters is None:
        res['depth_m'] = None
        return res
    sx, sy = scale_xy(float(res['cx']), float(res.get('cy', 0)), frame_shape, meters.shape)
    d = sample_m(meters, sx, sy)
    res['depth_m'] = None if d is None else round(d, 3)
    res['depth_cm'] = None if d is None else round(d * 100.0, 1)
    if d is not None:
        res['message'] = f"{res.get('message', '')} depth={res['depth_cm']:.0f}cm".strip()
    return res


def target_m(measured_m: float, params: Dict[str, Any]) -> float:
    """Final standoff in metres from first good depth + dashboard fields."""
    stop_at = float(params.get('stop_at_cm') or 0.0)
    approach = float(params.get('approach_cm') or 0.0)
    if stop_at > 0:
        return max(0.04, stop_at / 100.0)
    if approach > 0:
        return max(0.04, measured_m - approach / 100.0)
    return max(0.04, measured_m)
