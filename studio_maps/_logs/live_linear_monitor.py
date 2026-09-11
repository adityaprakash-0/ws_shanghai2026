#!/usr/bin/env python3
"""Live linear monitor: encoder + /oms/linear/status + oms.log + motion CSV."""
from __future__ import annotations

import json
import subprocess
import threading
import time
from datetime import datetime


def ts() -> str:
    return datetime.now().strftime('%H:%M:%S.%f')[:-3]


def run_echo(topic: str, handler) -> None:
    p = subprocess.Popen(
        ['ros2', 'topic', 'echo', topic],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        bufsize=1,
    )
    assert p.stdout is not None
    buf: list[str] = []
    for line in p.stdout:
        line = line.rstrip('\n')
        if line.strip() == '---':
            handler('\n'.join(buf))
            buf = []
        else:
            buf.append(line)


last_enc = [None, 0.0]


def on_enc(block: str) -> None:
    val = None
    for ln in block.splitlines():
        if 'data:' in ln:
            try:
                val = float(ln.split('data:', 1)[1].strip())
            except Exception:
                pass
    if val is None:
        return
    now = time.time()
    prev, t0 = last_enc
    if prev is None or abs(val - prev) >= 5.0 or (now - t0) >= 0.25:
        d = '' if prev is None else f'  d={val - prev:+.0f}'
        print(f'{ts()} ENC ticks={val:.1f}{d}', flush=True)
        last_enc[0], last_enc[1] = val, now


_last_stat = [0.0]


def on_status(block: str) -> None:
    raw = None
    for ln in block.splitlines():
        if 'data:' in ln:
            raw = ln.split('data:', 1)[1].strip().strip("'\"")
    if not raw:
        return
    try:
        d = json.loads(raw)
    except Exception:
        return
    now = time.time()
    # print every ~0.4s, or whenever state/speed changes meaningfully
    if now - _last_stat[0] < 0.35 and d.get('state') == 'idle' and abs(float(d.get('speed') or 0)) < 0.1:
        return
    _last_stat[0] = now
    print(
        f"{ts()} STAT state={d.get('state')} pos={d.get('pos_mm')} "
        f"ticks={d.get('ticks')} spd={d.get('speed')} "
        f"home_sw={d.get('home_switch')} homed={d.get('homed')}",
        flush=True,
    )


def tail(path: str, prefix: str, must_contain: str | None = None) -> None:
    p = subprocess.Popen(
        ['tail', '-n', '0', '-F', path],
        stdout=subprocess.PIPE,
        text=True,
        bufsize=1,
    )
    assert p.stdout is not None
    for line in p.stdout:
        line = line.rstrip()
        if must_contain and must_contain not in line:
            continue
        print(f'{ts()} {prefix} {line}', flush=True)


def main() -> None:
    print(f'{ts()} ready — ENC / STAT / LOG / CSV  (Ctrl+C to stop)', flush=True)
    threading.Thread(
        target=run_echo, args=('/titan1/m_3/encoder', on_enc), daemon=True).start()
    threading.Thread(
        target=run_echo, args=('/oms/linear/status', on_status), daemon=True).start()
    threading.Thread(
        target=tail,
        args=('/home/vmx/skill23_ws/studio_maps/_logs/oms.log', 'LOG', 'oms_linear'),
        daemon=True,
    ).start()
    threading.Thread(
        target=tail,
        args=('/home/vmx/skill23_ws/studio_maps/_logs/oms_linear_motion.csv', 'CSV'),
        daemon=True,
    ).start()
    while True:
        time.sleep(1.0)


if __name__ == '__main__':
    main()
