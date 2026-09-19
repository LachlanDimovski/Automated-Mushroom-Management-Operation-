#!/usr/bin/env python3
"""Serve ipad_teleop.html and proxy web_video_server through the same port, so
the iPad only ever needs http://<pc>:8000/ (stream + page on one origin).

    ros2 run web_video_server web_video_server      # port 8080
    ros2 run rosbridge_server rosbridge_websocket   # port 9090
    python3 ipad_teleop_server.py                   # port 8000 -> open http://<pc>:8000/
"""
import http.server
import os
import socketserver
import sys
import urllib.request

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
VIDEO = 'http://127.0.0.1:8080'
HERE = os.path.dirname(os.path.abspath(__file__))


class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *a, **k):
        super().__init__(*a, directory=HERE, **k)

    def do_GET(self):
        if self.path in ('/', ''):
            self.path = '/ipad_teleop.html'
        if self.path.startswith(('/stream', '/snapshot')):
            return self._proxy()
        return super().do_GET()

    def _proxy(self):
        try:
            with urllib.request.urlopen(VIDEO + self.path, timeout=10) as up:
                self.send_response(up.status)
                self.send_header('Content-Type', up.headers.get('Content-Type', 'application/octet-stream'))
                self.send_header('Cache-Control', 'no-cache')
                self.end_headers()
                while True:
                    chunk = up.read(16384)
                    if not chunk:
                        break
                    self.wfile.write(chunk)
        except (BrokenPipeError, ConnectionResetError):
            pass  # viewer went away
        except Exception as exc:
            self.send_error(502, f'video server: {exc}')

    def log_message(self, fmt, *args):
        if '/stream' not in (args[0] if args else ''):
            super().log_message(fmt, *args)


class Server(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True


if __name__ == '__main__':
    print(f'teleop page + video proxy on http://0.0.0.0:{PORT}/', flush=True)
    Server(('0.0.0.0', PORT), Handler).serve_forever()
