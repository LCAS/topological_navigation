"""Tests for the web app HTTP server (no ROS required)."""

import http.client
import json
import os

import pytest

from topological_navigation_visual.web_server import (
    DEFAULT_WEB_ROOT, WebAppServer, resolve_static_path,
)
from topological_navigation_visual.web_state import WebAppState


def _tmap():
    def node(name, x, edges):
        return {'node': {
            'name': name,
            'pose': {'position': {'x': x, 'y': 0.0},
                     'orientation': {'w': 1.0}},
            'edges': [{'edge_id': '%s_%s' % (name, t), 'node': t}
                      for t in edges],
        }}
    return {'name': 'm', 'nodes': [node('A', 0.0, ['B']),
                                   node('B', 1.0, ['A'])]}


@pytest.fixture
def server():
    state = WebAppState()
    state.set_map(_tmap())
    calls = []

    def on_goto(node):
        calls.append(('goto', node))
        state.goal_requested(node)
        return True, 'goal to %s sent' % node

    def on_cancel():
        calls.append(('cancel',))
        return False, 'not available'

    srv = WebAppServer(state, on_goto, on_cancel, host='127.0.0.1', port=0,
                       min_interval=0.01, keepalive=0.2)
    srv.calls = calls
    srv.start()
    yield srv
    srv.stop()


def _request(srv, method, path, body=None, headers=None):
    conn = http.client.HTTPConnection('127.0.0.1', srv.port, timeout=5)
    conn.request(method, path, body=body, headers=headers or {})
    resp = conn.getresponse()
    data = resp.read()
    conn.close()
    return resp, data


def _post_json(srv, path, payload):
    return _request(srv, 'POST', path, json.dumps(payload),
                    {'Content-Type': 'application/json'})


def test_bundled_assets_present():
    for rel in ('index.html', 'app.js', 'style.css',
                'vendor/react.production.min.js',
                'vendor/react-dom.production.min.js',
                'vendor/htm.umd.js', 'vendor/cytoscape.min.js'):
        assert os.path.isfile(os.path.join(DEFAULT_WEB_ROOT, rel)), rel


def test_index_uses_only_relative_urls():
    """Assets must resolve relatively so the app works behind a URL prefix."""
    with open(os.path.join(DEFAULT_WEB_ROOT, 'index.html')) as fh:
        index = fh.read()
    with open(os.path.join(DEFAULT_WEB_ROOT, 'app.js')) as fh:
        app = fh.read()
    for text in (index, app):
        assert 'http://' not in text and 'https://' not in text
        assert 'src="/' not in text and 'href="/' not in text
        assert "'/api/" not in text


def test_resolve_static_path_blocks_traversal(tmp_path):
    (tmp_path / 'index.html').write_text('x')
    assert resolve_static_path(str(tmp_path), '/') == \
        os.path.realpath(str(tmp_path / 'index.html'))
    assert resolve_static_path(str(tmp_path), '/../etc/passwd') is None
    assert resolve_static_path(str(tmp_path), '/%2e%2e/etc/passwd') is None
    assert resolve_static_path(str(tmp_path), '/missing.js') is None


def test_serves_index_and_vendor(server):
    resp, data = _request(server, 'GET', '/')
    assert resp.status == 200
    assert b'<div id="root">' in data
    resp, _ = _request(server, 'GET', '/vendor/react.production.min.js')
    assert resp.status == 200
    assert resp.getheader('Content-Type') == 'application/javascript'
    resp, _ = _request(server, 'GET', '/../setup.py')
    assert resp.status == 404


def test_state_and_map_endpoints(server):
    resp, data = _request(server, 'GET', '/api/map')
    assert resp.status == 200
    payload = json.loads(data)
    assert [n['name'] for n in payload['map']['nodes']] == ['A', 'B']
    resp, data = _request(server, 'GET', '/api/state')
    assert resp.status == 200
    assert json.loads(data)['navigation']['active'] is False


def test_goto(server):
    resp, data = _post_json(server, '/api/goto', {'node': 'B'})
    assert resp.status == 202
    assert json.loads(data)['ok'] is True
    assert server.calls == [('goto', 'B')]
    assert server.state.snapshot()['navigation']['destination'] == 'B'


def test_goto_validation(server):
    resp, _ = _post_json(server, '/api/goto', {'node': 'Z'})
    assert resp.status == 404
    resp, _ = _post_json(server, '/api/goto', {})
    assert resp.status == 400
    # Non-JSON content types are rejected (cross-site form posts).
    resp, _ = _request(server, 'POST', '/api/goto', 'node=A',
                       {'Content-Type': 'application/x-www-form-urlencoded'})
    assert resp.status == 400
    assert server.calls == []


def test_cancel_failure_reported(server):
    resp, data = _post_json(server, '/api/cancel', {})
    assert resp.status == 503
    assert json.loads(data) == {'ok': False, 'message': 'not available'}


def test_event_stream_pushes_map_and_updates(server):
    conn = http.client.HTTPConnection('127.0.0.1', server.port, timeout=5)
    conn.request('GET', '/api/events')
    resp = conn.getresponse()
    assert resp.status == 200
    assert resp.getheader('Content-Type') == 'text/event-stream'

    def next_event():
        event = None
        while True:
            line = resp.fp.readline().decode('utf-8').rstrip('\n')
            if line.startswith('event: '):
                event = line[len('event: '):]
            elif line.startswith('data: ') and event:
                return event, json.loads(line[len('data: '):])

    event, payload = next_event()
    assert event == 'map'
    assert payload['map']['name'] == 'm'
    event, payload = next_event()
    assert event == 'state'
    assert payload['robot']['current_node'] is None

    server.state.set_current_node('A')
    event, payload = next_event()
    assert event == 'state'
    assert payload['robot']['current_node'] == 'A'
    conn.close()
