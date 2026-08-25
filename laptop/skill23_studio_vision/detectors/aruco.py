"""ArUco detector. marker_id=-1 means any. Optional align uses image center."""

from __future__ import annotations

from typing import Any, Dict

import cv2

_DICT = None
_DET = None


def _setup():
    global _DICT, _DET
    if _DET is not None:
        return
    _DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    try:
        params = cv2.aruco.DetectorParameters()
        _DET = cv2.aruco.ArucoDetector(_DICT, params)
    except Exception:
        _DET = None


def detect(frame, params: Dict[str, Any]) -> Dict[str, Any]:
    _setup()
    want = int(params.get('marker_id', -1))
    if _DET is not None:
        corners, ids, _ = _DET.detectMarkers(frame)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(frame, _DICT)
    if ids is None or len(ids) == 0:
        return {'ok': False, 'message': 'no aruco'}
    ids = ids.flatten().tolist()
    pick = 0
    if want >= 0:
        if want not in ids:
            return {'ok': False, 'message': f'aruco {want} not in {ids}'}
        pick = ids.index(want)
    c = corners[pick][0]
    cx = float(c[:, 0].mean())
    cy = float(c[:, 1].mean())
    h, w = frame.shape[:2]
    return {
        'ok': True,
        'message': f'aruco id={ids[pick]}',
        'marker_id': int(ids[pick]),
        'ids': ids,
        'cx': cx,
        'cy': cy,
        'offset_x': (cx - w / 2.0) / max(1.0, w / 2.0),
        'offset_y': (cy - h / 2.0) / max(1.0, h / 2.0),
    }
