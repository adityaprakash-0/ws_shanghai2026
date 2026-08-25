"""HSV color blob detector. color_mode=any|named, color=red|…"""

from __future__ import annotations

from typing import Any, Dict, Optional, Tuple

import cv2
import numpy as np

# (lo, hi) in HSV; red wraps hue so it is two ranges
RANGES = {
    'red':    [((0, 80, 60), (10, 255, 255)), ((170, 80, 60), (180, 255, 255))],
    'orange': [((10, 80, 60), (25, 255, 255))],
    'yellow': [((22, 60, 60), (38, 255, 255))],
    'green':  [((40, 50, 40), (85, 255, 255))],
    'blue':   [((90, 50, 40), (130, 255, 255))],
    'purple': [((130, 40, 40), (160, 255, 255))],
    'white':  [((0, 0, 180), (180, 40, 255))],
    'black':  [((0, 0, 0), (180, 255, 50))],
}


def _mask(hsv: np.ndarray, name: str) -> np.ndarray:
    acc = None
    for lo, hi in RANGES[name]:
        m = cv2.inRange(hsv, np.array(lo), np.array(hi))
        acc = m if acc is None else cv2.bitwise_or(acc, m)
    return acc


def _biggest(mask: np.ndarray) -> Tuple[int, Optional[Tuple[int, int, int, int]]]:
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return 0, None
    c = max(cnts, key=cv2.contourArea)
    area = int(cv2.contourArea(c))
    if area < 400:
        return area, None
    return area, cv2.boundingRect(c)


def detect(frame, params: Dict[str, Any]) -> Dict[str, Any]:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mode = str(params.get('color_mode') or params.get('mode') or 'named').lower()
    wanted = str(params.get('color') or 'any').lower()
    if mode == 'any' or wanted in ('any', '*', 'all', ''):
        best_name, best_area, best_box = None, 0, None
        for name in RANGES:
            area, box = _biggest(_mask(hsv, name))
            if area > best_area:
                best_name, best_area, best_box = name, area, box
        if best_name is None:
            return {'ok': False, 'message': 'no color blob'}
        x, y, w, h = best_box
        fh, fw = frame.shape[:2]
        cx, cy = int(x + w / 2), int(y + h / 2)
        return {
            'ok': True,
            'message': f'color={best_name} area={best_area}',
            'color': best_name,
            'area': best_area,
            'cx': cx,
            'cy': cy,
            'offset_x': (cx - fw / 2.0) / max(1.0, fw / 2.0),
            'offset_y': (cy - fh / 2.0) / max(1.0, fh / 2.0),
        }
    if wanted not in RANGES:
        return {'ok': False, 'message': f'unknown color {wanted}'}
    area, box = _biggest(_mask(hsv, wanted))
    if box is None:
        return {'ok': False, 'message': f'{wanted} not found'}
    x, y, w, h = box
    fh, fw = frame.shape[:2]
    cx, cy = int(x + w / 2), int(y + h / 2)
    return {
        'ok': True,
        'message': f'color={wanted} area={area}',
        'color': wanted,
        'area': area,
        'cx': cx,
        'cy': cy,
        'offset_x': (cx - fw / 2.0) / max(1.0, fw / 2.0),
        'offset_y': (cy - fh / 2.0) / max(1.0, fh / 2.0),
    }
