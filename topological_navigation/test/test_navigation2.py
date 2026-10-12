"""Targeted tests for ``navigation2.py`` action-server behavior."""

from datetime import datetime

import pytest
import networkx as nx
from types import SimpleNamespace

from rclpy import Parameter
from rclpy.action import CancelResponse

from topological_navigation.navigation_graph import ActionSegment, NavState
from topological_navigation.scripts import navigation2 as navigation2_module
from topological_navigation.scripts.navigation2 import (
    TopologicalNavServer,
    _make_ros_param_value,
    _merge_ros_param_specs,
    _normalise_ros_param_spec,
    _ros_param_value,
)


class _DummyLogger:
    """Minimal logger stub for unit tests."""

    def debug(self, _msg):
        pass

    def info(self, _msg):
        pass

    def warning(self, _msg):
        pass

    def error(self, _msg):
        pass


class _DummyStateMachine:
    """Small state-machine stub used by action callback tests."""

    def __init__(self, terminal=False):
        self.terminal = terminal
        self.reset_called = False
        self.state = None
        self.transitions = []

    def transition(self, state):
        self.transitions.append(state)
        self.state = state
        return True

    def is_terminal(self):
        return self.terminal

    def reset(self):
        self.reset_called = True
        return True


class _FakeGotoGoalHandle:
    """Simple goal handle double for ``GotoNode`` callbacks."""

    def __init__(self, target='WP2', no_orientation=False):
        self.request = SimpleNamespace(
            target=target,
            no_orientation=no_orientation,
        )
        self.feedback = []
        self.final_state = None

    def publish_feedback(self, msg):
        self.feedback.append(msg)

    def succeed(self):
        self.final_state = 'succeeded'

    def abort(self):
        self.final_state = 'aborted'

    def canceled(self):
        self.final_state = 'canceled'


class _FakePolicyGoalHandle:
    """Simple goal handle double for ``ExecutePolicyMode`` callbacks."""

    def __init__(self, route):
        self.request = SimpleNamespace(route=route)
        self.feedback = []
        self.final_state = None

    def publish_feedback(self, msg):
        self.feedback.append(msg)

    def succeed(self):
        self.final_state = 'succeeded'

    def abort(self):
        self.final_state = 'aborted'

    def canceled(self):
        self.final_state = 'canceled'


class _FakePolicyGraph:
    """Small graph stub supporting get_route_edges-style access."""

    def __init__(self, edges):
        self._edges = list(edges)

    def has_edge(self, src, tgt):
        return (src, tgt) in self._edges

    def __contains__(self, node):
        return any(node in edge for edge in self._edges)

    def __getitem__(self, src):
        edge_map = {}
        for u, v in self._edges:
            if u != src:
                continue
            edge_map[v] = {
                'edge_id': '%s_%s' % (u, v),
                'source': u,
                'target': v,
                'properties': {},
            }
        return edge_map


def _make_server():
    """Create a partially constructed ``TopologicalNavServer`` for unit tests."""
    server = TopologicalNavServer.__new__(TopologicalNavServer)
    server.get_logger = lambda: _DummyLogger()
    server._sm = _DummyStateMachine()
    server._navigation_activated = False
    server._cancelled = False
    server._preempted = False
    server._no_orientation = False
    server._target = 'none'
    server._current_target = 'none'
    server._current_node = 'WP1'
    server._closest_node = 'WP1'
    server._tmap = {
        'nodes': [
            {
                'node': {
                    'name': 'WP1',
                    'edges': [
                        {'edge_id': 'WP1_WP2', 'node': 'WP2'},
                    ],
                },
            },
        ],
    }
    server._graph = _FakePolicyGraph([('WP1', 'WP2')])
    server._topol_map = 'test_map'
    server._global_metric_map_bounds = None
    server._global_metric_map = None
    server._map_actions = {}
    server._map_definitions = {}
    server._bt_files = {}
    server._action_clients = {}
    server._xy_tolerance_param = 'goal_checker.xy_goal_tolerance'
    server._yaw_tolerance_param = 'goal_checker.yaw_goal_tolerance'
    server._goal_checker_node = 'controller_server'
    server._param_clients = {}
    server._publish_status = lambda *_args, **_kwargs: None
    server._publish_route = lambda *_args, **_kwargs: None
    server._publish_route_segment_metric_map = (
        lambda *_args, **_kwargs: True
    )
    server._execute_route = lambda *_args, **_kwargs: True
    server._ensure_robot_within_main_map = lambda *_args, **_kwargs: True
    server._cancel_nav2_goal = lambda *_args, **_kwargs: None
    server._navigate = lambda *_args, **_kwargs: True
    return server


def _make_closest_edge_server(edges, closest_edge_ids, distances, target):
    """Create a server double for closest-edge route planning tests."""
    server = _make_server()
    server._current_node = 'none'
    server._closest_node = 'A'
    server._max_dist_to_closest_edge = 1.0
    server._closest_edges = SimpleNamespace(
        edge_ids=closest_edge_ids,
        distances=distances,
    )
    server._route_algorithm = 'astar'
    server._route_weight = 'weight'
    server._graph = nx.DiGraph()
    for node, x in [('A', 0.0), ('X', 1.0), ('B', 2.0), ('C', 3.0)]:
        server._graph.add_node(node, x=x, y=0.0)
    for source, destination, edge_id in edges:
        server._graph.add_edge(
            source, destination, edge_id=edge_id, weight=1.0,
        )
    server._test_target = target
    return server


def test_closest_edge_is_preserved_as_first_route_segment():
    """A threshold-qualified edge and its action must remain in the route."""
    server = _make_closest_edge_server(
        [
            ('A', 'X', 'A_X'),
            ('X', 'B', 'X_B'),
        ],
        ['A_X'],
        [0.1],
        'B',
    )

    route = server._route_from_closest_edge('B')

    assert route == ['A', 'X', 'B']


def test_bidirectional_closest_edge_selects_direction_toward_goal():
    """Equal-distance reverse edges choose the shorter continuation route."""
    server = _make_closest_edge_server(
        [
            ('A', 'B', 'A_B'),
            ('B', 'A', 'B_A'),
            ('B', 'C', 'B_C'),
        ],
        ['A_B', 'B_A'],
        [0.1, 0.1],
        'C',
    )

    route = server._route_from_closest_edge('C')

    assert route == ['A', 'B', 'C']


def test_closest_edge_outside_threshold_returns_no_forced_route():
    """The existing closest-node route fallback remains available."""
    server = _make_closest_edge_server(
        [('A', 'B', 'A_B')],
        ['A_B'],
        [1.1],
        'B',
    )

    assert server._route_from_closest_edge('B') is None


def test_cancel_goto_callback_accepts_cancel():
    """Cancel requests should be accepted and forwarded to Nav2."""
    server = _make_server()
    calls = []
    server._cancel_nav2_goal = lambda *_args, **_kwargs: calls.append('cancel')

    result = server._cancel_goto_cb(None)

    assert result == CancelResponse.ACCEPT
    assert server._cancelled is True
    assert calls == ['cancel']


def test_exec_goto_success_publishes_feedback_and_succeeds():
    """Successful ``GotoNode`` execution should publish feedback and succeed."""
    server = _make_server()
    goal_handle = _FakeGotoGoalHandle(target='WP2', no_orientation=True)

    result = server._exec_goto_cb(goal_handle)

    assert result.success is True
    assert goal_handle.final_state == 'succeeded'
    assert goal_handle.feedback
    assert goal_handle.feedback[0].route == 'Planning...'
    assert goal_handle.feedback[0].__class__.__name__.endswith('Feedback')


def test_exec_goto_cancelled_marks_goal_canceled():
    """Cancelled ``GotoNode`` executions should not be reported as aborted."""
    server = _make_server()
    goal_handle = _FakeGotoGoalHandle(target='WP2')

    def _cancelled_nav(_target):
        server._cancelled = True
        return False

    server._navigate = _cancelled_nav

    result = server._exec_goto_cb(goal_handle)

    assert result.success is False
    assert goal_handle.final_state == 'canceled'


def test_exec_policy_invalid_route_aborts_goal():
    """Invalid policy routes should abort the action instead of succeeding false."""
    server = _make_server()
    route = SimpleNamespace(source=['WP1', 'WP2'], edge_id=['edge_only_once'])
    goal_handle = _FakePolicyGoalHandle(route)

    result = server._exec_policy_cb(goal_handle)

    assert result.success is False
    assert goal_handle.final_state == 'aborted'


def test_exec_policy_success_enters_planning_and_sets_target():
    """Valid policy execution should enter PLANNING before running the route."""
    server = _make_server()
    route = SimpleNamespace(source=['WP1'], edge_id=['WP1_WP2'])
    goal_handle = _FakePolicyGoalHandle(route)
    observed = {}

    def _execute_route(route_nodes, target):
        observed['route_nodes'] = route_nodes
        observed['target'] = target
        observed['server_target'] = server._target
        return True

    server._execute_route = _execute_route

    result = server._exec_policy_cb(goal_handle)

    assert result.success is True
    assert goal_handle.final_state == 'succeeded'
    assert observed['route_nodes'] == ['WP1', 'WP2']
    assert observed['target'] == 'WP2'
    assert observed['server_target'] == 'WP2'
    assert NavState.PLANNING in server._sm.transitions


def test_load_map_config_clears_stale_action_clients_when_actions_missing():
    """Maps without ``actions`` should not retain stale clients from older maps."""
    server = _make_server()
    server._tmap = {}
    server._action_clients = {'stale': object()}

    server._load_map_config()

    assert server._action_clients == {}
    assert server._bt_files == {}


# =====================================================================
# ROS 2 parameter value helpers
# =====================================================================


class _FakeGraph:
    """Minimal networkx-compatible graph stub for unit tests."""

    def __init__(self, node_data):
        self.nodes = node_data

    def __contains__(self, item):
        return item in self.nodes


class TestRosParamHelpers:
    """Unit tests for _ros_param_value and _make_ros_param_value."""

    def _pv(self, value):
        """Round-trip: build ParameterValue then read it back."""
        from rcl_interfaces.msg import ParameterValue, ParameterType
        pv = _make_ros_param_value(value)
        return _ros_param_value(pv), pv

    def test_double_roundtrip(self):
        val, pv = self._pv(3.14)
        assert abs(val - 3.14) < 1e-9

    def test_int_roundtrip(self):
        val, pv = self._pv(42)
        assert val == 42

    def test_bool_roundtrip(self):
        val, pv = self._pv(True)
        assert val is True

    def test_string_roundtrip(self):
        val, pv = self._pv('hello')
        assert val == 'hello'

    def test_unsupported_type_returns_none(self):
        from rcl_interfaces.msg import ParameterValue
        # An unset ParameterValue has type 0 (PARAMETER_NOT_SET)
        pv = ParameterValue()
        assert _ros_param_value(pv) is None

    def test_make_unsupported_type_returns_unset(self):
        from rcl_interfaces.msg import ParameterType
        pv = _make_ros_param_value([1, 2, 3])  # list not supported
        assert pv.type == ParameterType.PARAMETER_NOT_SET


# =====================================================================
# set_ros_params spec normalisation
# =====================================================================


class TestNormaliseRosParamSpec:
    """Accepted authoring styles for ``set_ros_params``."""

    def test_none_returns_empty(self):
        assert _normalise_ros_param_spec(None, 'controller_server') == {}

    def test_node_keyed_mapping(self):
        spec = {'collision_monitor': {'FootprintApproach.enabled': False}}
        assert _normalise_ros_param_spec(spec, 'controller_server') == {
            'collision_monitor': {'FootprintApproach.enabled': False},
        }

    def test_flat_mapping_uses_default_node(self):
        spec = {'FollowPath.max_robot_speed': 0.3}
        assert _normalise_ros_param_spec(spec, 'controller_server') == {
            'controller_server': {'FollowPath.max_robot_speed': 0.3},
        }

    def test_list_entries_with_param_and_params(self):
        spec = [
            {'node': '/collision_monitor',
             'param': 'FootprintApproach.enabled', 'value': False},
            {'node': 'controller_server',
             'params': {'FollowPath.max_robot_speed': 0.3}},
            {'param': 'some.param', 'value': 1},
        ]
        assert _normalise_ros_param_spec(spec, 'controller_server') == {
            'collision_monitor': {'FootprintApproach.enabled': False},
            'controller_server': {
                'FollowPath.max_robot_speed': 0.3,
                'some.param': 1,
            },
        }

    def test_merge_override_wins_per_parameter(self):
        base = {'collision_monitor': {'a': 1, 'b': 2}}
        override = {'collision_monitor': {'b': 3}, 'other': {'c': 4}}
        merged = _merge_ros_param_specs(base, override)
        assert merged == {
            'collision_monitor': {'a': 1, 'b': 3},
            'other': {'c': 4},
        }
        assert base == {'collision_monitor': {'a': 1, 'b': 2}}


# =====================================================================
# _apply_segment_parameters / _restore_segment_parameters
# =====================================================================


class TestApplyRestoreSegmentParameters:
    """Parameter save/restore behaviour in isolation from Nav2."""

    def _make_segment(self, target='WP2', edge_props=None, action='navigate_to_pose'):
        """Build an ActionSegment stub."""
        props = edge_props if edge_props is not None else {}
        return ActionSegment(
            action_type=action,
            edge_ids=['e1'],
            source_nodes=['WP1'],
            target_nodes=[target],
            edge_data=[{'edge_id': 'e1', 'source': 'WP1',
                        'target': target, 'properties': props}],
        )

    def test_no_params_returns_empty(self):
        """Segment with no relevant properties -> no parameters set."""
        server = _make_server()
        # Use a dict-of-dicts with a nodes accessor (networkx-style)
        server._graph = _FakeGraph({'WP2': {'properties': {}, 'name': 'WP2'}})
        server._get_ros_params_sync = lambda names, node=None: {}
        server._set_ros_params_async = lambda _d, node=None: None

        seg = self._make_segment()
        prev = server._apply_segment_parameters(seg)
        assert prev == {}

    def test_node_tolerances_queried_and_set(self):
        """xy/yaw goal tolerances from node properties are queried and set."""
        server = _make_server()
        server._graph = _FakeGraph({
            'WP2': {
                'name': 'WP2',
                'properties': {
                    'xy_goal_tolerance': 0.1,
                    'yaw_goal_tolerance': 0.05,
                },
            },
        })
        queried_names = []
        set_calls = []

        def _fake_get(names, node=None):
            queried_names.extend(names)
            return {n: 0.5 for n in names}  # pretend current value is 0.5

        server._get_ros_params_sync = _fake_get
        server._set_ros_params_async = (
            lambda d, node=None: set_calls.append((node, dict(d)))
        )

        seg = self._make_segment()
        prev = server._apply_segment_parameters(seg)

        # Both tolerance param names should have been queried
        assert 'goal_checker.xy_goal_tolerance' in queried_names
        assert 'goal_checker.yaw_goal_tolerance' in queried_names

        # The returned dict contains the old values (0.5 faked above)
        gc = prev['controller_server']
        assert gc.get('goal_checker.xy_goal_tolerance') == 0.5
        assert gc.get('goal_checker.yaw_goal_tolerance') == 0.5

        # SetParameters was called with new values
        assert len(set_calls) == 1
        node, params = set_calls[0]
        assert node == 'controller_server'
        assert params['goal_checker.xy_goal_tolerance'] == pytest.approx(0.1)
        assert params['goal_checker.yaw_goal_tolerance'] == pytest.approx(0.05)

    def test_edge_ros_parameters_queried_and_set(self):
        """Edge properties mapped via ros_parameters are queried and applied."""
        server = _make_server()
        server._graph = _FakeGraph({})  # no node in graph -> no tolerance params
        server._action_clients = {
            'row_traversal': {
                'client': None,
                'action_class': None,
                'config': {
                    'ros_parameters': {
                        'max_speed': 'FollowPath.max_robot_speed',
                    },
                },
            },
        }

        queried_names = []
        set_calls = []

        def _fake_get(names, node=None):
            queried_names.extend(names)
            return {'FollowPath.max_robot_speed': 1.0}

        server._get_ros_params_sync = _fake_get
        server._set_ros_params_async = (
            lambda d, node=None: set_calls.append((node, dict(d)))
        )

        seg = self._make_segment(
            target='WP2',
            edge_props={'max_speed': 0.3},
            action='row_traversal',
        )
        prev = server._apply_segment_parameters(seg)

        assert 'FollowPath.max_robot_speed' in queried_names
        assert prev['controller_server'].get(
            'FollowPath.max_robot_speed') == pytest.approx(1.0)
        assert set_calls[0][1]['FollowPath.max_robot_speed'] == pytest.approx(0.3)

    def test_action_set_ros_params_applied_to_other_node(self):
        """Action-level set_ros_params targets an arbitrary node."""
        server = _make_server()
        server._graph = _FakeGraph({})
        server._action_clients = {
            'row_traversal': {
                'config': {
                    'set_ros_params': {
                        'collision_monitor': {
                            'FootprintApproach.enabled': False,
                        },
                    },
                },
            },
        }
        set_calls = []
        server._get_ros_params_sync = (
            lambda names, node=None: {n: True for n in names}
        )
        server._set_ros_params_async = (
            lambda d, node=None: set_calls.append((node, dict(d)))
        )

        seg = self._make_segment(action='row_traversal')
        prev = server._apply_segment_parameters(seg)

        assert set_calls == [
            ('collision_monitor', {'FootprintApproach.enabled': False}),
        ]
        assert prev == {
            'collision_monitor': {'FootprintApproach.enabled': True},
        }

    def test_edge_set_ros_params_overrides_action_level(self):
        """Edge-level set_ros_params wins over the action-level default."""
        server = _make_server()
        server._graph = _FakeGraph({})
        server._action_clients = {
            'row_traversal': {
                'config': {
                    'set_ros_params': {
                        'collision_monitor': {
                            'FootprintApproach.enabled': False,
                            'polygon_slow.action_type': 'slowdown',
                        },
                    },
                },
            },
        }
        set_calls = []
        server._get_ros_params_sync = lambda names, node=None: {}
        server._set_ros_params_async = (
            lambda d, node=None: set_calls.append((node, dict(d)))
        )

        seg = self._make_segment(
            action='row_traversal',
            edge_props={
                'set_ros_params': [
                    {
                        'node': '/collision_monitor',
                        'param': 'FootprintApproach.enabled',
                        'value': True,
                    },
                ],
            },
        )
        server._apply_segment_parameters(seg)

        assert len(set_calls) == 1
        node, params = set_calls[0]
        assert node == 'collision_monitor'
        assert params['FootprintApproach.enabled'] is True
        assert params['polygon_slow.action_type'] == 'slowdown'

    def test_restore_calls_set_with_prev_values(self):
        """_restore_segment_parameters sends saved values via SetParameters."""
        server = _make_server()
        set_calls = []
        server._set_ros_params_async = (
            lambda d, node=None: set_calls.append((node, dict(d)))
        )

        prev = {
            'controller_server': {
                'goal_checker.xy_goal_tolerance': 0.5,
                'goal_checker.yaw_goal_tolerance': 0.2,
            },
            'collision_monitor': {'FootprintApproach.enabled': True},
        }
        server._restore_segment_parameters(prev)

        assert len(set_calls) == 2
        assert dict(set_calls) == prev

    def test_restore_empty_is_noop(self):
        """_restore_segment_parameters({}) must not call SetParameters."""
        server = _make_server()
        set_calls = []
        server._set_ros_params_async = lambda d, node=None: set_calls.append(d)

        server._restore_segment_parameters({})
        assert set_calls == []

    def test_get_ros_params_sync_unavailable_service(self):
        """_get_ros_params_sync returns {} when service is not ready."""
        server = _make_server()

        class _FakeClient:
            def service_is_ready(self):
                return False

        server._get_params_client = _FakeClient()
        server._set_params_client = _FakeClient()

        result = server._get_ros_params_sync(['some_param'])
        assert result == {}

    def test_get_ros_params_sync_empty_names(self):
        """_get_ros_params_sync with empty list returns {} immediately."""
        server = _make_server()
        # No client needed – should return before touching it
        result = server._get_ros_params_sync([])
        assert result == {}


# ----------------------------------------------------------------------
# Dynamic (runtime) parameter callback
# ----------------------------------------------------------------------
def _param(name, value, ptype):
    """Build a parameter double exposing name/value/type_."""
    return SimpleNamespace(name=name, value=value, type_=ptype)


def test_parameters_callback_updates_route_algorithm():
    """A valid algorithm change should be applied and accepted."""
    server = _make_server()
    result = server._parameters_callback(
        [_param('route_algorithm', 'dijkstra', Parameter.Type.STRING)]
    )
    assert result.successful is True
    assert server._route_algorithm == 'dijkstra'


def test_parameters_callback_rejects_unknown_algorithm():
    """An unknown algorithm must be rejected without mutating state."""
    server = _make_server()
    server._route_algorithm = 'astar'
    result = server._parameters_callback(
        [_param('route_algorithm', 'bfs', Parameter.Type.STRING)]
    )
    assert result.successful is False
    assert server._route_algorithm == 'astar'


def test_parameters_callback_rejects_negative_max_dist():
    """A negative origin distance must be rejected."""
    server = _make_server()
    result = server._parameters_callback(
        [_param('max_dist_to_closest_edge', -1.0, Parameter.Type.DOUBLE)]
    )
    assert result.successful is False


def test_parameters_callback_rejects_empty_weight_attr():
    """An empty weight attribute name must be rejected."""
    server = _make_server()
    result = server._parameters_callback(
        [_param('route_weight_attr', '', Parameter.Type.STRING)]
    )
    assert result.successful is False


def test_parameters_callback_updates_numeric_and_weight_params():
    """Valid numeric/string updates should all be applied."""
    server = _make_server()
    result = server._parameters_callback([
        _param('max_dist_to_closest_edge', 2.5, Parameter.Type.DOUBLE),
        _param('coarse_white_extension_m', 4.7, Parameter.Type.DOUBLE),
        _param('route_white_extension_m', 2.8, Parameter.Type.DOUBLE),
        _param('route_weight_attr', 'cost', Parameter.Type.STRING),
    ])
    assert result.successful is True
    assert server._max_dist_to_closest_edge == 2.5
    assert server._coarse_white_extension_m == 4.7
    assert server._route_white_extension_m == 2.8
    assert server._route_weight == 'cost'


def test_parameters_callback_updates_metric_map_parameters():
    """Metric map generation parameters should accept valid updates."""
    server = _make_server()
    result = server._parameters_callback([
        _param('metric_map_resolution', 1.0, Parameter.Type.DOUBLE),
        _param('route_segment_resolution', 0.05, Parameter.Type.DOUBLE),
        _param('route_segment_border_width', 0.3, Parameter.Type.DOUBLE),
        _param('route_segment_padding', 0.2, Parameter.Type.DOUBLE),
    ])
    assert result.successful is True
    assert server._metric_map_resolution == 1.0
    assert server._route_segment_resolution == 0.05
    assert server._route_segment_border_width == 0.3
    assert server._route_segment_padding == 0.2


def test_parameters_callback_rejects_invalid_metric_map_values():
    """Invalid metric map values should be rejected."""
    server = _make_server()

    result = server._parameters_callback([
        _param('metric_map_resolution', 0.0, Parameter.Type.DOUBLE),
    ])
    assert result.successful is False

    result = server._parameters_callback([
        _param('route_segment_resolution', -0.1, Parameter.Type.DOUBLE),
    ])
    assert result.successful is False

    result = server._parameters_callback([
        _param('route_segment_border_width', -0.1, Parameter.Type.DOUBLE),
    ])
    assert result.successful is False

    result = server._parameters_callback([
        _param('route_segment_padding', -0.1, Parameter.Type.DOUBLE),
    ])
    assert result.successful is False

    result = server._parameters_callback([
        _param('coarse_white_extension_m', -0.1, Parameter.Type.DOUBLE),
    ])
    assert result.successful is False

    result = server._parameters_callback([
        _param('route_white_extension_m', -0.1, Parameter.Type.DOUBLE),
    ])
    assert result.successful is False


@pytest.mark.parametrize(
    ('robot_position', 'expected_route_start', 'expected_publish_count'),
    [
        ((3.5, -1.25), (3.5, -1.25), 1),
        ((5.0, -1.25), None, 0),
    ],
)
def test_route_segment_map_validates_robot_position(
    monkeypatch, robot_position, expected_route_start, expected_publish_count,
):
    """Do not create a local metric map when the robot is outside."""
    server = _make_server()
    server._global_metric_map_bounds = (0.0, -2.0, 5.0, 2.0)
    server._global_metric_map = SimpleNamespace(
        info=SimpleNamespace(
            resolution=1.0,
            width=5,
            height=4,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=-2.0),
            ),
        ),
        data=[0] * 20,
    )
    server._route_white_extension_m = 2.0
    server._route_segment_border_width = 0.25
    server._route_segment_resolution = 0.05
    server._route_segment_padding = 0.0
    server._base_frame = 'base_link'
    server._publish_status = lambda *_args, **_kwargs: None
    del server._ensure_robot_within_main_map
    del server._publish_route_segment_metric_map
    server._raster_to_occupancy_grid = lambda raster, frame: (raster, frame)

    server._robot_position_in_frame = (
        lambda frame_id: robot_position
    )
    published = []
    server._route_segment_map_pub = SimpleNamespace(
        publish=lambda msg: published.append(msg),
    )

    geometry = SimpleNamespace(frame_id='map')
    raster = SimpleNamespace(
        image=SimpleNamespace(width=10, height=20),
        resolution=0.05,
    )
    captured = {}
    monkeypatch.setattr(
        navigation2_module,
        'geometry_from_tmap',
        lambda tmap, apply_transform: geometry,
    )
    monkeypatch.setattr(
        navigation2_module,
        'route_specs_from_edge_data',
        lambda edges, default_left_m, default_right_m: ('spec',),
    )

    def _capture_rasterize(_geometry, _specs, **kwargs):
        captured.update(kwargs)
        return raster

    monkeypatch.setattr(
        navigation2_module,
        'rasterize_route_geometry',
        _capture_rasterize,
    )

    server._publish_route_segment_metric_map([{'source': 'A', 'target': 'B'}])

    assert captured.get('route_start') == expected_route_start
    assert len(published) == expected_publish_count


def test_navigate_outside_main_map_does_not_plan_or_publish(monkeypatch):
    """An outside robot must be rejected before route planning starts."""
    server = _make_server()
    server._ensure_robot_within_main_map = lambda: False
    server._determine_origin = lambda target: 'WP1'
    server._publish_route_segment_metric_map = lambda edges: pytest.fail(
        'local metric map must not be created',
    )
    monkeypatch.setattr(
        navigation2_module,
        'plan_route',
        lambda *args, **kwargs: pytest.fail('route must not be planned'),
    )

    result = TopologicalNavServer._navigate(server, 'WP2')

    assert result is False
    assert NavState.FAILED in server._sm.transitions


def test_outside_main_map_publishes_explicit_status():
    """The operator-facing status should identify the containment failure."""
    server = _make_server()
    del server._ensure_robot_within_main_map
    server._global_metric_map_bounds = (0.0, -2.0, 5.0, 2.0)
    server._global_metric_map = SimpleNamespace(
        info=SimpleNamespace(
            resolution=1.0,
            width=5,
            height=4,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=-2.0),
            ),
        ),
        data=[0] * 20,
    )
    statuses = []
    server._publish_status = statuses.append

    result = server._ensure_robot_within_main_map('map', (5.0, 0.0))

    assert result is False
    assert statuses == ['OUTSIDE_MAIN_MAP']


@pytest.mark.parametrize('occupancy', [-1, 100])
def test_unknown_or_occupied_main_map_cell_is_outside(occupancy):
    """Only a known-free coarse-map cell can authorize navigation."""
    server = _make_server()
    server._global_metric_map = SimpleNamespace(
        info=SimpleNamespace(
            resolution=1.0,
            width=2,
            height=2,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0),
            ),
        ),
        data=[occupancy, 0, 0, 0],
    )

    assert server._position_within_global_metric_map((0.5, 0.5)) is False
    assert server._position_within_global_metric_map((1.5, 0.5)) is True


def test_policy_outside_main_map_does_not_create_local_map():
    """A supplied policy route is also rejected when the robot is outside."""
    server = _make_server()
    server._ensure_robot_within_main_map = lambda: False
    server._publish_route_segment_metric_map = lambda edges: pytest.fail(
        'local metric map must not be created',
    )
    server._execute_route = lambda *args: pytest.fail(
        'route must not be executed',
    )
    route = SimpleNamespace(source=['WP1'], edge_id=['WP1_WP2'])
    goal_handle = _FakePolicyGoalHandle(route)

    result = server._exec_policy_cb(goal_handle)

    assert result.success is False
    assert goal_handle.final_state == 'aborted'


def _make_segment_graph(nodes):
    """Build a real DiGraph with x positions, one edge per consecutive node."""
    graph = nx.DiGraph()
    for i, node in enumerate(nodes):
        graph.add_node(node, x=float(i), y=0.0)
    for u, v in zip(nodes, nodes[1:]):
        graph.add_edge(u, v, weight=1.0)
    return graph


def _make_stats_server(nodes, current_node):
    """Server double wired up for ``_log_segment_edges``/``_execute_segment`` tests."""
    server = _make_server()
    server._graph = _make_segment_graph(nodes)
    server._current_node = current_node
    server._map_hash = 'test-hash'
    server._stat = None
    server._pending_segment = None
    server._published = []
    server._publish_stats = lambda: server._published.append(server._stat)
    return server


def _make_test_segment(nodes, edge_ids):
    return ActionSegment(
        action_type='NavigateToPose',
        edge_ids=list(edge_ids),
        source_nodes=list(nodes[:-1]),
        target_nodes=list(nodes[1:]),
        edge_data=[{} for _ in edge_ids],
    )


def test_log_segment_edges_success_logs_every_edge():
    """A fully successful segment gets one 'success' row per edge."""
    nodes = ['WP1', 'WP2', 'WP3', 'WP4']
    server = _make_stats_server(nodes, current_node='WP4')
    segment = _make_test_segment(nodes, ['e12', 'e23', 'e34'])
    start, end = datetime.now(), datetime.now()

    server._log_segment_edges(segment, True, 'none', start, end)

    assert [s.edge_id for s in server._published] == ['e12', 'e23', 'e34']
    assert all(s.status == 'success' for s in server._published)
    assert all(s.is_segment for s in server._published)
    assert all(s.segment_edge_ids == ['e12', 'e23', 'e34'] for s in server._published)


def test_log_segment_edges_partial_failure_leaves_remaining_edges_unlogged():
    """Completed edges are logged success, the failing edge failed, the rest absent."""
    nodes = ['WP1', 'WP2', 'WP3', 'WP4', 'WP5']
    # Robot got as far as WP2 (1 completed edge) before the segment failed.
    server = _make_stats_server(nodes, current_node='WP2')
    segment = _make_test_segment(nodes, ['e12', 'e23', 'e34', 'e45'])
    start, end = datetime.now(), datetime.now()

    server._log_segment_edges(segment, False, 'aborted_by_costmap', start, end)

    assert [s.edge_id for s in server._published] == ['e12', 'e23']
    assert server._published[0].status == 'success'
    assert server._published[1].status == 'failed'
    assert server._published[1].failure_reason == 'aborted_by_costmap'


def test_log_segment_edges_cancelled_marks_boundary_edge_aborted():
    """A cancelled segment marks the in-flight edge 'aborted', not 'failed'."""
    nodes = ['WP1', 'WP2', 'WP3']
    server = _make_stats_server(nodes, current_node='WP1')
    segment = _make_test_segment(nodes, ['e12', 'e23'])
    start, end = datetime.now(), datetime.now()

    server._log_segment_edges(segment, False, 'cancelled', start, end)

    assert len(server._published) == 1
    assert server._published[0].edge_id == 'e12'
    assert server._published[0].status == 'aborted'


def test_execute_segment_exception_logs_failure_and_returns_false():
    """An unhandled exception mid-goal still yields a DB row, not a crash."""
    nodes = ['WP1', 'WP2']
    server = _make_stats_server(nodes, current_node='WP1')
    segment = _make_test_segment(nodes, ['e12'])
    segment.edge_data = [{'source': 'WP1', 'target': 'WP2', 'edge_id': 'e12'}]
    server._tmap = {
        'nodes': [{'node': {'name': 'WP1', 'edges': [
            {'edge_id': 'e12', 'node': 'WP2'},
        ]}}],
    }
    server._topol_map = 'test_map'
    server._action_clients = {'NavigateToPose': {'client': object()}}
    server._apply_segment_parameters = lambda _seg: {}
    server._restore_segment_parameters = lambda _prev: None
    server._build_segment_goal = lambda _seg, _is_final: object()
    server._publish_current_edge = lambda *_a, **_kw: None
    server._publish_segment_boundary = lambda *_a, **_kw: None
    server._goal_reached = False

    def _raise(*_a, **_kw):
        raise RuntimeError('nav2 client exploded')

    server._send_nav2_goal = _raise

    result = server._execute_segment(segment, True, 0, 1)

    assert result is False
    assert server._pending_segment is None
    assert len(server._published) == 1
    assert server._published[0].status == 'failed'
    assert 'nav2 client exploded' in server._published[0].failure_reason


def test_on_shutdown_flushes_pending_segment():
    """A segment still in flight at shutdown is logged instead of dropped."""
    nodes = ['WP1', 'WP2']
    server = _make_stats_server(nodes, current_node='WP1')
    segment = _make_test_segment(nodes, ['e12'])
    server._pending_segment = (segment, datetime.now())
    server._navigation_activated = False
    server._stats_db = None

    server._on_shutdown()

    assert server._pending_segment is None
    assert len(server._published) == 1
    assert server._published[0].status == 'failed'
    assert server._published[0].failure_reason == 'shutdown'
