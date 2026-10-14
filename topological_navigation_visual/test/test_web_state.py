"""Tests for the web app backend state store (no ROS required)."""

import math
import threading

from topological_navigation_visual.web_state import (
    WebAppState,
    is_roboflow_enabled,
    route_edge_ids,
    tmap_to_graph,
    yaw_from_quaternion,
)


def _node(name, x, y, edges=(), properties=None):
    node = {
        'name': name,
        'pose': {
            'position': {'x': x, 'y': y, 'z': 0.0},
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
        },
        'edges': [
            {'edge_id': '%s_%s' % (name, t), 'node': t,
             'action': 'NavigateToPose'}
            for t in edges
        ],
    }
    if properties is not None:
        node['properties'] = properties
    return {'meta': {'node': name}, 'node': node}


def _tmap():
    return {
        'name': 'test_map',
        'transformation': {'topo_frame_id': 'map'},
        'nodes': [
            _node('A', 0.0, 0.0, ['B']),
            _node('B', 1.0, 0.0, ['A', 'C'],
                  properties={'roboflow': {'enabled': True}, 'field': 1}),
            _node('C', 2.0, 1.0, ['B', 'MISSING']),
        ],
    }


def test_roboflow_detection():
    assert is_roboflow_enabled({'roboflow': {'enabled': True}})
    assert is_roboflow_enabled({'roboflow': {'enabled': 'true'}})
    assert is_roboflow_enabled({'roboflow': True})
    assert not is_roboflow_enabled({'roboflow': {'enabled': False}})
    assert not is_roboflow_enabled({'roboflow': {}})
    assert not is_roboflow_enabled({})
    assert not is_roboflow_enabled(None)


def test_yaw_from_quaternion():
    assert yaw_from_quaternion(0, 0, 0, 1) == 0.0
    s = math.sin(math.pi / 4)
    assert math.isclose(yaw_from_quaternion(0, 0, s, s), math.pi / 2)


def test_tmap_to_graph():
    graph = tmap_to_graph(_tmap())
    assert graph['name'] == 'test_map'
    assert graph['frame'] == 'map'
    nodes = {n['name']: n for n in graph['nodes']}
    assert set(nodes) == {'A', 'B', 'C'}
    assert nodes['B']['roboflow'] is True
    assert nodes['A']['roboflow'] is False
    assert nodes['B']['properties']['field'] == 1
    assert nodes['C']['x'] == 2.0 and nodes['C']['y'] == 1.0
    # Edge to a node not in the map is dropped.
    pairs = {(e['source'], e['target']) for e in graph['edges']}
    assert pairs == {('A', 'B'), ('B', 'A'), ('B', 'C'), ('C', 'B')}


def test_route_edge_ids():
    graph = tmap_to_graph(_tmap())
    assert route_edge_ids(['A', 'B', 'C'], graph['edges']) == ['A_B', 'B_C']
    assert route_edge_ids([], graph['edges']) == []
    assert route_edge_ids(['A'], graph['edges']) == []


def test_set_map_bumps_versions_only_on_change():
    state = WebAppState()
    assert state.set_map(_tmap())
    v, mv = state.version, state.map_version
    assert not state.set_map(_tmap())
    assert (state.version, state.map_version) == (v, mv)
    assert state.has_node('A') and not state.has_node('Z')
    assert state.map_frame == 'map'


def test_robot_state_snapshot():
    state = WebAppState()
    state.set_current_node('A')
    state.set_closest_node('none')
    state.set_pose(1.23456, -2.0, 0.5, 'map')
    state.set_odometry(0.3, 0.4, 0.1)
    robot = state.snapshot()['robot']
    assert robot['current_node'] == 'A'
    assert robot['closest_node'] is None
    assert robot['pose'] == {'x': 1.235, 'y': -2.0, 'yaw': 0.5,
                             'frame': 'map'}
    assert robot['speed'] == {'linear': 0.5, 'angular': 0.1}
    v = state.version
    state.set_current_node('A')
    assert state.version == v  # unchanged value -> no update


def test_web_goal_lifecycle_and_route_highlight():
    state = WebAppState(clock=lambda: 42.0)
    state.set_map(_tmap())
    state.set_route(['A', 'B', 'C'])
    # Route is only reported while navigating.
    assert state.snapshot()['navigation']['route'] == []

    state.goal_requested('C')
    nav = state.snapshot()['navigation']
    assert nav['active'] and nav['destination'] == 'C'

    state.goal_accepted('C')
    state.set_route(['A', 'B', 'C'])
    state.set_move_status({'goal': 'B', 'final_goal': 'C'})
    state.set_current_edge('A_B--test_map')
    nav = state.snapshot()['navigation']
    assert nav['route'] == ['A', 'B', 'C']
    assert nav['route_edges'] == ['A_B', 'B_C']
    assert nav['current_goal'] == 'B'
    assert nav['current_edge'] == 'A_B'

    state.goal_finished('C', 'succeeded')
    state.set_action_statuses([4])
    nav = state.snapshot()['navigation']
    assert not nav['active']
    assert nav['destination'] is None
    assert nav['route_edges'] == []
    assert nav['last_result'] == {'target': 'C', 'outcome': 'succeeded',
                                  'time': 42.0}


def test_preempted_goal_does_not_clear_new_goal():
    state = WebAppState()
    state.set_map(_tmap())
    state.goal_requested('C')
    state.goal_accepted('C')
    state.goal_requested('A')
    state.goal_finished('C', 'aborted')
    nav = state.snapshot()['navigation']
    assert nav['active'] and nav['destination'] == 'A'


def test_external_goal_via_action_status():
    state = WebAppState()
    state.set_action_statuses([2])
    state.set_move_status({'goal': 'B', 'final_goal': 'C'})
    nav = state.snapshot()['navigation']
    assert nav['active']
    assert nav['destination'] == 'C' and nav['current_goal'] == 'B'
    state.set_action_statuses([4, 5])
    assert not state.snapshot()['navigation']['active']


def test_wait_for_change_wakes_up():
    state = WebAppState()
    v = state.version
    timer = threading.Timer(0.05, state.set_current_node, args=('A',))
    timer.start()
    assert state.wait_for_change(v, timeout=2.0) != v
    timer.join()
    # Times out without change.
    v = state.version
    assert state.wait_for_change(v, timeout=0.01) == v
