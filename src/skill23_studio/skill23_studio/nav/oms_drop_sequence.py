#!/usr/bin/env python3
"""OMS drop sequence for commissioning Module G.

Current behaviour (stub until competition drop is specified):
  1. Caller aligns robot yaw to the drop waypoint (AMCL, ±1°).
  2. This module / detect_align mode ``oms_drop`` brings OMS to full home.

Replace ``run_drop_sequence`` body later with the real place/open/retract flow.
Do not change the commissioning mission state machine when editing this file —
only the OMS steps here.
"""

from __future__ import annotations

import json
import time
from typing import Any, Callable, Dict, Optional, Tuple


def run_drop_sequence(
        *,
        home_cmd_pub,
        wait_home: Callable[[float], Tuple[bool, str]],
        log: Optional[Callable[[str], None]] = None,
        timeout_sec: float = 180.0,
) -> Tuple[bool, str, Dict[str, Any]]:
    """Home OMS at the drop location. Real place sequence TBD."""
    _log = log or (lambda _m: None)
    _log('DROP stub — OMS full home (place sequence TBD)')
    payload = {'mode': 'home', 'cmd_id': f'drop_home_{int(time.time() * 1000)}'}
    try:
        from std_msgs.msg import String
        home_cmd_pub.publish(String(data=json.dumps(payload)))
    except Exception as exc:
        return False, f'drop home publish failed: {exc}', {}
    ok, msg = wait_home(timeout_sec)
    extra = {'drop_ok': bool(ok), 'drop_mode': 'oms_home_stub'}
    if ok:
        _log(f'DROP stub done — OMS homed ({msg})')
        return True, 'drop complete (oms home stub)', extra
    return False, f'drop OMS home failed: {msg}', extra
