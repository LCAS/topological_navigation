#!/usr/bin/env python3
# Copyright (c) 2026, topological_navigation contributors
# Licensed under the MIT License.
"""Single-port HTTP server for the topological navigation web app.

Serves the bundled (offline) React front end and a small JSON API:

``GET  /api/state``   full UI state snapshot (robot + navigation)
``GET  /api/map``     current topological map graph
``GET  /api/events``  Server-Sent Events stream pushing ``map`` and
                      ``state`` events to every connected client
``POST /api/goto``    ``{"node": "<name>"}`` -- send the robot to a node
``POST /api/cancel``  cancel the current navigation goal

Only the Python standard library is used, so no extra dependencies are
required on the robot.
"""

import json
import mimetypes
import os
import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Callable, Optional, Tuple
from urllib.parse import unquote, urlparse

from topological_navigation_visual.web_state import WebAppState

DEFAULT_PORT = 8083
DEFAULT_WEB_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'web')
MAX_BODY_BYTES = 4096

# Callback signature: returns (success, message)
ActionCallback = Callable[..., Tuple[bool, str]]


def resolve_static_path(web_root: str, url_path: str) -> Optional[str]:
    """Map a URL path to a file under ``web_root``.

    Returns None if the file does not exist or the path would escape the
    web root (directory traversal).
    """
    rel = unquote(url_path).lstrip('/')
    if rel == '':
        rel = 'index.html'
    root = os.path.realpath(web_root)
    full = os.path.realpath(os.path.join(root, rel))
    if os.path.commonpath([root, full]) != root:
        return None
    if not os.path.isfile(full):
        return None
    return full


class WebAppServer:
    """Threaded HTTP server exposing a :class:`WebAppState` to browsers."""

    def __init__(self, state: WebAppState, on_goto: ActionCallback,
                 on_cancel: ActionCallback, host: str = '0.0.0.0',
                 port: int = DEFAULT_PORT, web_root: str = DEFAULT_WEB_ROOT,
                 min_interval: float = 0.1, keepalive: float = 15.0,
                 logger=None):
        self.state = state
        self.on_goto = on_goto
        self.on_cancel = on_cancel
        self.web_root = web_root
        self.min_interval = float(min_interval)
        self.keepalive = float(keepalive)
        self.logger = logger
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

        handler = _make_handler(self)
        self._httpd = ThreadingHTTPServer((host, int(port)), handler)
        self._httpd.daemon_threads = True

    @property
    def port(self) -> int:
        return self._httpd.server_address[1]

    @property
    def stopping(self) -> bool:
        return self._stop.is_set()

    def start(self) -> None:
        self._thread = threading.Thread(
            target=self._httpd.serve_forever, name='topo_web_app_http',
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self.state.notify_all()
        self._httpd.shutdown()
        self._httpd.server_close()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def log_error(self, msg: str) -> None:
        if self.logger is not None:
            self.logger.error(msg)


def _make_handler(server: WebAppServer):
    """Create a request handler class bound to ``server``."""

    class _Handler(BaseHTTPRequestHandler):
        server_version = 'TopologicalWebApp/1.0'

        # Silence default stderr access logging.
        def log_message(self, format, *args):  # noqa: A002
            pass

        # ---------------------------------------------------------- helpers
        def _send_json(self, payload, status=HTTPStatus.OK):
            body = json.dumps(payload).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def _read_json(self):
            ctype = self.headers.get('Content-Type', '')
            # Requiring JSON forces a CORS pre-flight for cross-origin
            # browser requests, which this server never approves.
            if ctype.split(';')[0].strip().lower() != 'application/json':
                raise ValueError('Content-Type must be application/json')
            try:
                length = int(self.headers.get('Content-Length', '0'))
            except ValueError:
                raise ValueError('invalid Content-Length')
            if length < 0 or length > MAX_BODY_BYTES:
                raise ValueError('request body too large')
            raw = self.rfile.read(length) if length else b'{}'
            data = json.loads(raw.decode('utf-8') or '{}')
            if not isinstance(data, dict):
                raise ValueError('JSON body must be an object')
            return data

        # ------------------------------------------------------------- GET
        def do_GET(self):  # noqa: N802
            path = urlparse(self.path).path
            if path == '/api/state':
                self._send_json(server.state.snapshot())
            elif path == '/api/map':
                self._send_json(server.state.map_snapshot())
            elif path == '/api/events':
                self._stream_events()
            elif path.startswith('/api/'):
                self._send_json({'error': 'not found'}, HTTPStatus.NOT_FOUND)
            else:
                self._serve_static(path)

        def _serve_static(self, path):
            full = resolve_static_path(server.web_root, path)
            if full is None:
                self._send_json({'error': 'not found'}, HTTPStatus.NOT_FOUND)
                return
            ctype = mimetypes.guess_type(full)[0] or 'application/octet-stream'
            if full.endswith('.js'):
                ctype = 'application/javascript'
            with open(full, 'rb') as fh:
                body = fh.read()
            self.send_response(HTTPStatus.OK)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            self.wfile.write(body)

        def _write_event(self, event, payload):
            data = json.dumps(payload)
            self.wfile.write(
                ('event: %s\ndata: %s\n\n' % (event, data)).encode('utf-8'))
            self.wfile.flush()

        def _stream_events(self):
            self.send_response(HTTPStatus.OK)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Connection', 'keep-alive')
            self.send_header('X-Accel-Buffering', 'no')
            self.end_headers()
            state = server.state
            sent_map_version = None
            try:
                self.wfile.write(b'retry: 2000\n\n')
                while not server.stopping:
                    snapshot = state.snapshot()
                    if snapshot['map_version'] != sent_map_version:
                        map_snap = state.map_snapshot()
                        self._write_event('map', map_snap)
                        sent_map_version = map_snap['map_version']
                    self._write_event('state', snapshot)
                    # Rate-limit updates for all clients.
                    if server._stop.wait(server.min_interval):
                        break
                    new_version = state.wait_for_change(
                        snapshot['version'], server.keepalive)
                    if new_version == snapshot['version'] \
                            and not server.stopping:
                        self.wfile.write(b': keepalive\n\n')
                        self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError,
                    ConnectionAbortedError, TimeoutError):
                pass

        # ------------------------------------------------------------ POST
        def do_POST(self):  # noqa: N802
            path = urlparse(self.path).path
            try:
                data = self._read_json()
            except (ValueError, UnicodeDecodeError) as exc:
                self._send_json({'ok': False, 'message': str(exc)},
                                HTTPStatus.BAD_REQUEST)
                return

            if path == '/api/goto':
                node = data.get('node')
                if not isinstance(node, str) or not node:
                    self._send_json(
                        {'ok': False, 'message': "missing 'node'"},
                        HTTPStatus.BAD_REQUEST)
                    return
                if not server.state.has_node(node):
                    self._send_json(
                        {'ok': False, 'message': 'unknown node %r' % node},
                        HTTPStatus.NOT_FOUND)
                    return
                self._dispatch(server.on_goto, node)
            elif path == '/api/cancel':
                self._dispatch(server.on_cancel)
            else:
                self._send_json({'error': 'not found'}, HTTPStatus.NOT_FOUND)

        def _dispatch(self, callback, *args):
            try:
                ok, message = callback(*args)
            except Exception as exc:  # pragma: no cover - defensive
                server.log_error('web app request failed: %s' % exc)
                ok, message = False, 'internal error'
            status = HTTPStatus.ACCEPTED if ok \
                else HTTPStatus.SERVICE_UNAVAILABLE
            self._send_json({'ok': ok, 'message': message}, status)

    return _Handler
