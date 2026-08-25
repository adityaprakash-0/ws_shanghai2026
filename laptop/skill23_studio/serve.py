#!/usr/bin/env python3
"""Static server for the Studio dashboard with caching disabled.

The dashboard is edited constantly during a competition; a browser holding on
to an old app.js has already caused "phantom" behaviour that no longer exists
in the source. Serving everything as no-store removes that class of confusion.
"""

from __future__ import annotations

import os
import sys
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer


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

    def log_message(self, fmt: str, *args) -> None:
        if '"GET /api' in (fmt % args):
            return
        super().log_message(fmt, *args)


def main() -> None:
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 3003
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    server = ThreadingHTTPServer(('0.0.0.0', port), NoCacheHandler)
    print(f'Skill23 Studio → http://127.0.0.1:{port}  (no-cache)')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
