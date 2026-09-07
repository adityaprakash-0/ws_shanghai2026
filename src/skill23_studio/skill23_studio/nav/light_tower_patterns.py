# Light-tower / wait-light pattern catalog for Navigation WP actions.
# Hardware: studica light_tower via /light_tower/set (SetData.params string).
# Each pattern is a list of (command, dwell_sec). Ends with off when played.
# Colors: red / green / yellow / buzzer (no blue LED).

from __future__ import annotations

from typing import Dict, List, Tuple

Step = Tuple[str, float]

_PATTERNS: Dict[str, List[Step]] = {
    'off': [('off', 0.05)],
    'red': [('red', 1.0)],
    'green': [('green', 1.0)],
    'yellow': [('yellow', 1.0)],
    'buzzer': [('buzzer', 0.8), ('off', 0.2)],
    'red_blink': [('red:blink', 2.0)],
    'green_blink': [('green:blink', 2.0)],
    'yellow_blink': [('yellow:blink', 2.0)],
    'red_blink_3x': [
        ('red', 0.35), ('off', 0.25),
        ('red', 0.35), ('off', 0.25),
        ('red', 0.35), ('off', 0.25),
    ],
    'green_blink_3x': [
        ('green', 0.35), ('off', 0.25),
        ('green', 0.35), ('off', 0.25),
        ('green', 0.35), ('off', 0.25),
    ],
    'yellow_blink_3x': [
        ('yellow', 0.35), ('off', 0.25),
        ('yellow', 0.35), ('off', 0.25),
        ('yellow', 0.35), ('off', 0.25),
    ],
    'buzzer_3x': [
        ('buzzer', 0.25), ('off', 0.25),
        ('buzzer', 0.25), ('off', 0.25),
        ('buzzer', 0.25), ('off', 0.25),
    ],
}


def list_patterns() -> List[str]:
    return sorted(_PATTERNS.keys())


def get_pattern(name: str) -> List[Step]:
    key = str(name or 'red_blink_3x').strip().lower()
    return list(_PATTERNS.get(key, _PATTERNS['red_blink_3x']))


def pattern_duration(name: str) -> float:
    return sum(max(0.0, float(d)) for _, d in get_pattern(name))
