#!/usr/bin/env python3
"""Static server for the Studio dashboard with caching disabled.

Also accepts POST /api/health-log so the laptop can archive Nav2 / stack
health lines locally (Pi stays light — it only streams telemetry).
"""

from __future__ import annotations

import json
import os
import sys
import time
from datetime import datetime, timezone
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse


ROOT = Path(__file__).resolve().parent
HEALTH_DIR = ROOT / 'health_logs'


class NoCacheHandler(SimpleHTTPRequestHandler):
    def end_headers(self) -> None:
        self.send_header('Cache-Control', 'no-store, must-revalidate')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Expires', '0')
        super().end_headers()

    def send_head(self):
        # Never answer with 304; always hand back the current file.
        for header in ('If-Modified-Since', 'If-None-Match'):
            while header in self.headers:
                del self.headers[header]
        return super().send_head()

    def _json(self, code: int, payload: dict) -> None:
        body = json.dumps(payload).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path != '/api/health-log':
            self.send_error(404, 'Not found')
            return
        length = int(self.headers.get('Content-Length') or 0)
        raw = self.rfile.read(length) if length > 0 else b'{}'
        try:
            data = json.loads(raw.decode('utf-8') or '{}')
        except json.JSONDecodeError:
            self._json(400, {'ok': False, 'error': 'invalid json'})
            return
        try:
            HEALTH_DIR.mkdir(parents=True, exist_ok=True)
            session = str(data.get('session') or 'studio').strip() or 'studio'
            safe = ''.join(c if c.isalnum() or c in '-_' else '_' for c in session)[:64]
            day = datetime.now(timezone.utc).strftime('%Y%m%d')
            path = HEALTH_DIR / f'{safe}_{day}.jsonl'
            row = {
                't': data.get('t') or time.time(),
                'iso': datetime.now(timezone.utc).isoformat(),
                'runtime_s': data.get('runtime_s'),
                'map': data.get('map'),
                'robot': data.get('robot'),
                'rows': data.get('rows') or [],
            }
            with path.open('a', encoding='utf-8') as fh:
                fh.write(json.dumps(row, ensure_ascii=False) + '\n')
            self._json(200, {'ok': True, 'path': str(path)})
        except OSError as exc:
            self._json(500, {'ok': False, 'error': str(exc)})

    def log_message(self, fmt: str, *args) -> None:
        msg = fmt % args
        if '"GET /api' in msg or '"POST /api/health-log' in msg:
            return
        super().log_message(fmt, *args)


def main() -> None:
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 3003
    os.chdir(str(ROOT))
    server = ThreadingHTTPServer(('0.0.0.0', port), NoCacheHandler)
    print(f'Skill23 Studio → http://127.0.0.1:{port}  (no-cache)')
    print(f'Health logs → {HEALTH_DIR}')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
