"""YOLO detector. Copy trained weights to models/best.pt (or YOLO_MODEL)."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Dict, Optional

_MODEL = None
_ERR = ''


def _model_path() -> Path:
    env = os.environ.get('YOLO_MODEL', '')
    if env:
        return Path(env).expanduser()
    here = Path(__file__).resolve().parent.parent / 'models'
    for name in ('best.pt', 'yolo.pt', 'yolov8n.pt'):
        p = here / name
        if p.is_file():
            return p
    return here / 'best.pt'


def _load():
    global _MODEL, _ERR
    if _MODEL is not None or _ERR:
        return
    path = _model_path()
    if not path.is_file():
        _ERR = f'YOLO model missing: {path} (copy trained .pt here)'
        return
    try:
        from ultralytics import YOLO
        _MODEL = YOLO(str(path))
    except Exception as exc:
        _ERR = f'ultralytics load failed: {exc}'


def detect(frame, params: Dict[str, Any]) -> Dict[str, Any]:
    _load()
    if _MODEL is None:
        return {'ok': False, 'message': _ERR or 'yolo unavailable'}
    want = str(params.get('class_name') or 'any').lower()
    conf = float(params.get('conf', 0.4))
    res = _MODEL.predict(frame, verbose=False, conf=conf)[0]
    names = res.names or {}
    hits = []
    if res.boxes is not None:
        for b in res.boxes:
            cls_id = int(b.cls[0])
            name = str(names.get(cls_id, cls_id)).lower()
            score = float(b.conf[0])
            xyxy = [float(v) for v in b.xyxy[0].tolist()]
            hits.append({'class': name, 'conf': round(score, 3), 'box': xyxy})
    if not hits:
        return {'ok': False, 'message': 'no yolo detections'}
    if want not in ('', 'any', '*', 'all'):
        hits = [h for h in hits if h['class'] == want]
        if not hits:
            return {'ok': False, 'message': f'class {want} not found'}
    best = max(hits, key=lambda h: h['conf'])
    x1, y1, x2, y2 = best['box']
    fh, fw = frame.shape[:2]
    cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    return {
        'ok': True,
        'message': f"yolo {best['class']} {best['conf']}",
        'class_name': best['class'],
        'conf': best['conf'],
        'cx': cx,
        'cy': cy,
        'offset_x': (cx - fw / 2.0) / max(1.0, fw / 2.0),
        'offset_y': (cy - fh / 2.0) / max(1.0, fh / 2.0),
        'hits': hits[:8],
    }
