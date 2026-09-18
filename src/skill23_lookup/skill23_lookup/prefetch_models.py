#!/usr/bin/env python3
"""Prefetch EasyOCR models so competition runs work offline.

Models land in ~/.EasyOCR/model/ (craft_mlt_25k.pth + english_g2.pth).
ArUco needs no download (OpenCV bundled dictionaries).
"""

from __future__ import annotations

import os
import sys
from pathlib import Path


def _model_dir() -> Path:
    return Path.home() / '.EasyOCR' / 'model'


def models_present() -> dict:
    d = _model_dir()
    craft = d / 'craft_mlt_25k.pth'
    eng = d / 'english_g2.pth'
    return {
        'dir': str(d),
        'craft': craft.is_file(),
        'english_g2': eng.is_file(),
        'craft_bytes': craft.stat().st_size if craft.is_file() else 0,
        'english_bytes': eng.stat().st_size if eng.is_file() else 0,
    }


def ensure_easyocr_models(*, force_download: bool = False) -> dict:
    info = models_present()
    if info['craft'] and info['english_g2'] and not force_download:
        return {'ok': True, 'downloaded': False, **info}

    # Creating the Reader downloads missing weights into ~/.EasyOCR/model.
    import easyocr  # noqa: WPS433

    print('[skill23_lookup] Loading EasyOCR Reader (may download on first run)…',
          flush=True)
    # gpu=False: Pi / Jetson competition boxes often have no CUDA for EasyOCR.
    _ = easyocr.Reader(['en'], gpu=False, verbose=False)
    info = models_present()
    ok = bool(info['craft'] and info['english_g2'])
    return {'ok': ok, 'downloaded': True, **info}


def warm_aruco() -> dict:
    import cv2  # noqa: WPS433

    if not hasattr(cv2, 'aruco'):
        return {'ok': False, 'error': 'cv2.aruco missing (need opencv-contrib)'}
    dict_id = getattr(cv2.aruco, 'DICT_4X4_50', None)
    if dict_id is None:
        return {'ok': False, 'error': 'DICT_4X4_50 missing'}
    dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
    params = cv2.aruco.DetectorParameters()
    try:
        _ = cv2.aruco.ArucoDetector(dictionary, params)
    except Exception as exc:  # noqa: BLE001
        return {'ok': False, 'error': str(exc)}
    return {'ok': True, 'dict': 'DICT_4X4_50'}


def main(argv: list[str] | None = None) -> int:
    del argv
    force = os.environ.get('SKILL23_LOOKUP_FORCE_DOWNLOAD', '').lower() in (
        '1', 'true', 'yes')
    print('[skill23_lookup] prefetch / verify models', flush=True)
    ocr = ensure_easyocr_models(force_download=force)
    print(f'  EasyOCR: {ocr}', flush=True)
    aruco = warm_aruco()
    print(f'  ArUco:   {aruco}', flush=True)
    if not ocr.get('ok') or not aruco.get('ok'):
        print('[skill23_lookup] FAILED — missing models or OpenCV ArUco',
              file=sys.stderr, flush=True)
        return 1
    print('[skill23_lookup] OK — offline-ready', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
