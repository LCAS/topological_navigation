"""Targeted tests for ``navigation2.py`` action-server behavior."""

from types import SimpleNamespace

from rclpy.action import CancelResponse

from topological_navigation.navigation_graph import NavState
from topological_navigation.scripts.navigation2 import TopologicalNavServer


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
    server._graph = {}
    server._topol_map = 'test_map'
    server._map_actions = {}
    server._map_definitions = {}
    server._bt_files = {}
    server._action_clients = {}
    server._publish_status = lambda *_args, **_kwargs: None
    server._publish_route = lambda *_args, **_kwargs: None
    server._execute_route = lambda *_args, **_kwargs: True
    server._cancel_nav2_goal = lambda *_args, **_kwargs: None
    server._navigate = lambda *_args, **_kwargs: True
    return server


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
    assert goal_handle.feedback[0].__class__.__name__.endswith('_Feedback')


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
