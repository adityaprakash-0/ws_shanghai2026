"""OCR via pytesseract (optional). params.text = substring to match."""

from __future__ import annotations

from typing import Any, Dict


def detect(frame, params: Dict[str, Any]) -> Dict[str, Any]:
    try:
        import pytesseract
    except Exception:
        return {'ok': False, 'message': 'pytesseract not installed'}
    text = pytesseract.image_to_string(frame) or ''
    text = ' '.join(text.split())
    want = str(params.get('text') or '').strip().lower()
    if not text:
        return {'ok': False, 'message': 'ocr empty'}
    if want and want not in text.lower():
        return {'ok': False, 'message': f'ocr missing "{want}"'}
    fh, fw = frame.shape[:2]
    cx, cy = fw / 2.0, fh / 2.0
    try:
        import pytesseract as _pt
        data = _pt.image_to_data(frame, output_type=_pt.Output.DICT)
        n = len(data.get('text') or [])
        for i in range(n):
            word = (data['text'][i] or '').strip()
            if not word:
                continue
            if want and want not in word.lower() and want not in text.lower():
                continue
            cx = float(data['left'][i] + data['width'][i] / 2.0)
            cy = float(data['top'][i] + data['height'][i] / 2.0)
            if not want or want in word.lower():
                break
    except Exception:
        pass
    return {
        'ok': True,
        'message': f'ocr: {text[:80]}',
        'text': text,
        'cx': cx,
        'cy': cy,
        'offset_x': (cx - fw / 2.0) / max(1.0, fw / 2.0),
        'offset_y': (cy - fh / 2.0) / max(1.0, fh / 2.0),
    }
