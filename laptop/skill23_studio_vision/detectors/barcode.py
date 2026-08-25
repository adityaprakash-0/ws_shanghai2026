"""1D barcode via pyzbar (optional)."""

from __future__ import annotations

from typing import Any, Dict

import cv2


def detect(frame, params: Dict[str, Any]) -> Dict[str, Any]:
    try:
        from pyzbar.pyzbar import decode
    except Exception:
        return {'ok': False, 'message': 'pyzbar not installed'}
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found = decode(gray)
    if not found:
        return {'ok': False, 'message': 'no barcode'}
    codes = []
    fh, fw = frame.shape[:2]
    cx = fw / 2.0
    cy = fh / 2.0
    for f in found:
        try:
            data = f.data.decode('utf-8')
        except Exception:
            data = str(f.data)
        codes.append({'type': str(f.type), 'data': data})
        r = getattr(f, 'rect', None)
        if r is not None:
            cx = float(r.left + r.width / 2.0)
            cy = float(r.top + r.height / 2.0)
    return {
        'ok': True,
        'message': f"barcode {codes[0]['type']}: {codes[0]['data']}",
        'codes': codes,
        'cx': cx,
        'cy': cy,
        'offset_x': (cx - fw / 2.0) / max(1.0, fw / 2.0),
        'offset_y': (cy - fh / 2.0) / max(1.0, fh / 2.0),
    }
