#!/usr/bin/env python3
"""WorldSkills Module G — pick/drop commissioning helpers."""

from __future__ import annotations

import re
from typing import Any, Dict, List, Optional, Tuple

COMMISSION_OBJECTS = ('Cylinder', 'Semisphere', 'Rectangle', 'Mystery')

_STICKER_RE = re.compile(r'^(pick|p|drop|d)(\d+)$', re.IGNORECASE)


def default_config() -> Dict[str, Any]:
    return {
        'drop_assignments': {obj: '' for obj in COMMISSION_OBJECTS},
        'detect_retries': 3,
        'yaw_tolerance_deg': 1.0,
        # Optional overrides keyed by sticker (pick1…). Waypoint fields win if set.
        'pick_counts': {},
        'pick_lifts_mm': {},
    }


def normalize_config(raw: Any) -> Dict[str, Any]:
    base = default_config()
    if not isinstance(raw, dict):
        return base
    drops = raw.get('drop_assignments')
    if isinstance(drops, dict):
        for obj in COMMISSION_OBJECTS:
            val = str(drops.get(obj) or '').strip().lower()
            base['drop_assignments'][obj] = val
    try:
        base['detect_retries'] = max(
            1, min(10, int(raw.get('detect_retries', base['detect_retries']))))
    except (TypeError, ValueError):
        pass
    try:
        base['yaw_tolerance_deg'] = max(
            0.1, min(15.0, float(raw.get('yaw_tolerance_deg', 1.0))))
    except (TypeError, ValueError):
        pass
    for key in ('pick_counts', 'pick_lifts_mm'):
        src = raw.get(key)
        if isinstance(src, dict):
            out: Dict[str, Any] = {}
            for k, v in src.items():
                label = str(k or '').strip().lower()
                if not label:
                    continue
                try:
                    out[label] = float(v) if key == 'pick_lifts_mm' else int(v)
                except (TypeError, ValueError):
                    continue
            base[key] = out
    return base


def normalize_sticker(raw: Any) -> Tuple[str, int]:
    """Return (kind, number): kind is pick|drop|home|''."""
    s = str(raw or '').strip().lower()
    if not s:
        return '', 0
    if s in ('home', 'start', 'finish'):
        return 'home', 0
    m = _STICKER_RE.match(s)
    if not m:
        return '', 0
    kind = 'pick' if m.group(1).lower() in ('pick', 'p') else 'drop'
    return kind, int(m.group(2))


def sticker_label(kind: str, num: int) -> str:
    if kind == 'home':
        return 'home'
    if kind == 'pick':
        return f'pick{num}'
    if kind == 'drop':
        return f'drop{num}'
    return ''


def _safe_int(raw: Any, default: int) -> int:
    try:
        return int(raw)
    except (TypeError, ValueError):
        return default


def _safe_float(raw: Any, default: float) -> float:
    try:
        return float(raw)
    except (TypeError, ValueError):
        return default


def pick_count_of(
        wp: Dict[str, Any], label: str, config: Optional[Dict[str, Any]] = None,
) -> int:
    """Visits remaining for this pick (≥1). Waypoint pick_count overrides config."""
    cfg = config or {}
    overrides = cfg.get('pick_counts') or {}
    if isinstance(wp, dict) and wp.get('pick_count') is not None:
        return max(1, min(20, _safe_int(wp.get('pick_count'), 1)))
    if isinstance(overrides, dict) and label in overrides:
        return max(1, min(20, _safe_int(overrides.get(label), 1)))
    return 1


def pick_lift_mm_of(
        wp: Dict[str, Any], label: str, config: Optional[Dict[str, Any]] = None,
) -> float:
    """Lift mm down from home for this pick. Waypoint pick_lift_mm overrides config."""
    cfg = config or {}
    overrides = cfg.get('pick_lifts_mm') or {}
    if isinstance(wp, dict) and wp.get('pick_lift_mm') is not None:
        return max(0.0, min(300.0, _safe_float(wp.get('pick_lift_mm'), 0.0)))
    if isinstance(overrides, dict) and label in overrides:
        return max(0.0, min(300.0, _safe_float(overrides.get(label), 0.0)))
    return 0.0


def index_waypoints(
        waypoints: List[Dict[str, Any]],
        config: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    picks: List[Tuple[int, Dict[str, Any]]] = []
    drops: Dict[str, Dict[str, Any]] = {}
    home: Optional[Dict[str, Any]] = None
    labels: List[str] = []
    counts: Dict[str, int] = {}
    lifts: Dict[str, float] = {}
    for wp in waypoints:
        if not isinstance(wp, dict):
            continue
        kind, num = normalize_sticker(wp.get('sticker'))
        if kind == 'pick':
            picks.append((num, wp))
        elif kind == 'drop':
            label = sticker_label('drop', num)
            drops[label] = wp
            drops[f'd{num}'] = wp
        elif kind == 'home':
            home = wp
    picks.sort(key=lambda item: item[0])
    pick_wps: List[Dict[str, Any]] = []
    for num, wp in picks:
        label = sticker_label('pick', num)
        labels.append(label)
        pick_wps.append(wp)
        counts[label] = pick_count_of(wp, label, config)
        lifts[label] = pick_lift_mm_of(wp, label, config)
    return {
        'picks': pick_wps,
        'pick_labels': labels,
        'pick_counts': counts,
        'pick_lifts_mm': lifts,
        'drops': drops,
        'home': home,
    }


def remaining_after_visit(
        remaining: Dict[str, int], label: str,
) -> Dict[str, int]:
    out = dict(remaining or {})
    cur = max(0, int(out.get(label, 0)) - 1)
    out[label] = cur
    return out


def next_pick_index(
        labels: List[str],
        remaining: Dict[str, int],
        after_index: int,
) -> Optional[int]:
    """Next pick with remaining > 0, searching after ``after_index`` then wrap."""
    n = len(labels)
    if n <= 0:
        return None
    start = max(-1, int(after_index))
    for step in range(1, n + 1):
        idx = (start + step) % n
        label = labels[idx]
        if int(remaining.get(label, 0)) > 0:
            return idx
    return None


def normalize_object_name(raw: Any) -> str:
    s = str(raw or '').strip()
    if not s:
        return ''
    low = s.lower()
    for obj in COMMISSION_OBJECTS:
        if low == obj.lower():
            return obj
    aliases = {
        'semi': 'Semisphere',
        'semisphere': 'Semisphere',
        'hemisphere': 'Semisphere',
        'rect': 'Rectangle',
        'cyl': 'Cylinder',
    }
    return aliases.get(low, s)


def drop_sticker_for_object(
        config: Dict[str, Any], class_name: str) -> str:
    obj = normalize_object_name(class_name)
    drops = config.get('drop_assignments') or {}
    if not isinstance(drops, dict):
        return ''
    return str(drops.get(obj) or '').strip().lower()


def resolve_drop_wp(
        drops: Dict[str, Dict[str, Any]], sticker: str) -> Optional[Dict[str, Any]]:
    s = str(sticker or '').strip().lower()
    if not s:
        return None
    if s in drops:
        return drops[s]
    kind, num = normalize_sticker(s)
    if kind == 'drop':
        return drops.get(sticker_label('drop', num))
    return None
