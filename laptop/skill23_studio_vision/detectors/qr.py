"""QR via OpenCV QRCodeDetector."""

from __future__ import annotations

from typing import Any, Dict

import cv2

_DET = cv2.QRCodeDetector()


def detect(frame, params: Dict[str, Any]) -> Dict[str, Any]:
    data, pts, _ = _DET.detectAndDecode(frame)
    if not data:
        return {'ok': False, 'message': 'no qr'}
    fh, fw = frame.shape[:2]
    cx, cy = fw / 2.0, fh / 2.0
    if pts is not None and len(pts):
        p = pts.reshape(-1, 2)
        cx = float(p[:, 0].mean())
        cy = float(p[:, 1].mean())
    return {
        'ok': True,
        'message': f'qr: {data[:80]}',
        'text': data,
        'cx': cx,
        'cy': cy,
        'offset_x': (cx - fw / 2.0) / max(1.0, fw / 2.0),
        'offset_y': (cy - fh / 2.0) / max(1.0, fh / 2.0),
    }
