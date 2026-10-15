"""
Pytest configuration for topological_navigation tests.

This file ensures that the topological_navigation package can be imported
during testing by adding the source directory to the Python path and
invalidating any stale module cache.
"""

import sys
import types
from pathlib import Path

# The ROS package directory containing the actual Python package.
# Layout: topological_navigation/ (ROS pkg) -> topological_navigation/ (python pkg)
_pkg_dir = str(Path(__file__).resolve().parent.parent)

# Insert BEFORE anything else to win over the outer __init__.py
if _pkg_dir not in sys.path:
    sys.path.insert(0, _pkg_dir)

# Invalidate the root cached module so Python re-discovers from the correct path.
# Only remove the root module – submodules will be imported fresh.
# We must not remove test.conftest itself.
_stale = 'topological_navigation'
if _stale in sys.modules:
    _cached_file = getattr(sys.modules[_stale], '__file__', '') or ''
    # Only invalidate if the cached module points to the wrong location
    if 'topological_navigation/topological_navigation' not in _cached_file:
        del sys.modules[_stale]


def _install_ros_test_stubs():
    """Install minimal ROS 2 module stubs for pure unit-test collection."""

    class _BaseMsg:
        def __init__(self, **kwargs):
            for key, value in kwargs.items():
                setattr(self, key, value)

    def _module(name):
        module = types.ModuleType(name)
        sys.modules[name] = module
        return module

    # action_msgs.msg -------------------------------------------------
    action_msgs = _module('action_msgs')
    action_msgs.msg = _module('action_msgs.msg')

    class GoalStatus:
        STATUS_UNKNOWN = 0
        STATUS_ACCEPTED = 1
        STATUS_EXECUTING = 2
        STATUS_CANCELING = 3
        STATUS_SUCCEEDED = 4
        STATUS_CANCELED = 5
        STATUS_ABORTED = 6

    action_msgs.msg.GoalStatus = GoalStatus

    # geometry_msgs.msg ----------------------------------------------
    geometry_msgs = _module('geometry_msgs')
    geometry_msgs.msg = _module('geometry_msgs.msg')

    class Point32(_BaseMsg):
        def __init__(self, x=0.0, y=0.0, z=0.0):
            super().__init__(x=x, y=y, z=z)

    class PoseStamped(_BaseMsg):
        def __init__(self):
            super().__init__(
                header=types.SimpleNamespace(frame_id='', stamp=None),
                pose=types.SimpleNamespace(
                    position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                    orientation=types.SimpleNamespace(
                        x=0.0, y=0.0, z=0.0, w=1.0,
                    ),
                ),
            )

    class Pose(_BaseMsg):
        def __init__(self):
            super().__init__(
                position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=types.SimpleNamespace(
                    x=0.0, y=0.0, z=0.0, w=1.0,
                ),
            )

    class PolygonStamped(_BaseMsg):
        def __init__(self):
            super().__init__(
                header=types.SimpleNamespace(frame_id='', stamp=None),
                polygon=types.SimpleNamespace(points=[]),
            )

    geometry_msgs.msg.Point32 = Point32
    geometry_msgs.msg.Pose = Pose
    geometry_msgs.msg.PolygonStamped = PolygonStamped
    geometry_msgs.msg.PoseStamped = PoseStamped

    # nav_msgs.msg ---------------------------------------------------
    nav_msgs = _module('nav_msgs')
    nav_msgs.msg = _module('nav_msgs.msg')

    class OccupancyGrid(_BaseMsg):
        def __init__(self):
            super().__init__(
                header=types.SimpleNamespace(frame_id='', stamp=None),
                info=types.SimpleNamespace(
                    resolution=0.0,
                    width=0,
                    height=0,
                    origin=types.SimpleNamespace(
                        position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                        orientation=types.SimpleNamespace(
                            x=0.0, y=0.0, z=0.0, w=1.0,
                        ),
                    ),
                ),
                data=[],
            )

    nav_msgs.msg.OccupancyGrid = OccupancyGrid

    # rcl_interfaces --------------------------------------------------
    rcl_interfaces = _module('rcl_interfaces')
    rcl_interfaces.msg = _module('rcl_interfaces.msg')
    rcl_interfaces.srv = _module('rcl_interfaces.srv')

    class ParameterType:
        PARAMETER_NOT_SET = 0
        PARAMETER_BOOL = 1
        PARAMETER_INTEGER = 2
        PARAMETER_DOUBLE = 3
        PARAMETER_STRING = 4
        PARAMETER_BYTE_ARRAY = 5
        PARAMETER_BOOL_ARRAY = 6
        PARAMETER_INTEGER_ARRAY = 7
        PARAMETER_DOUBLE_ARRAY = 8
        PARAMETER_STRING_ARRAY = 9

    class ParameterValue(_BaseMsg):
        def __init__(self, **kwargs):
            self.type = ParameterType.PARAMETER_NOT_SET
            self.bool_value = False
            self.integer_value = 0
            self.double_value = 0.0
            self.string_value = ''
            for key, value in kwargs.items():
                setattr(self, key, value)

    class RclParameter(_BaseMsg):
        pass

    class SetParameters:
        class Request(_BaseMsg):
            def __init__(self):
                super().__init__(parameters=[])

        class Response(_BaseMsg):
            def __init__(self):
                super().__init__(results=[])

    class GetParameters:
        class Request(_BaseMsg):
            def __init__(self):
                super().__init__(names=[])

        class Response(_BaseMsg):
            def __init__(self):
                super().__init__(values=[])

    class SetParametersResult(_BaseMsg):
        def __init__(self, successful=False, reason=''):
            super().__init__(successful=successful, reason=reason)

    rcl_interfaces.msg.Parameter = RclParameter
    rcl_interfaces.msg.ParameterType = ParameterType
    rcl_interfaces.msg.ParameterValue = ParameterValue
    rcl_interfaces.msg.SetParametersResult = SetParametersResult
    rcl_interfaces.srv.SetParameters = SetParameters
    rcl_interfaces.srv.GetParameters = GetParameters

    # rclpy -----------------------------------------------------------
    rclpy = _module('rclpy')
    rclpy.action = _module('rclpy.action')
    rclpy.callback_groups = _module('rclpy.callback_groups')
    rclpy.executors = _module('rclpy.executors')
    rclpy.node = _module('rclpy.node')
    rclpy.qos = _module('rclpy.qos')

    class Node:
        pass

    class Parameter:
        class Type:
            STRING = 1
            DOUBLE = 2
            BOOL = 3
            INTEGER = 4

    class _ActionEndpoint:
        def __init__(self, *args, **kwargs):
            pass

    class CancelResponse:
        ACCEPT = 1
        REJECT = 2

    class _Group:
        def __init__(self, *args, **kwargs):
            pass

    class MultiThreadedExecutor:
        def __init__(self, *args, **kwargs):
            pass

    class _QoSValue:
        def __init__(self, value):
            self.value = value

    class QoSProfile:
        def __init__(self, *args, **kwargs):
            pass

    rclpy.Parameter = Parameter
    rclpy.init = lambda *args, **kwargs: None
    rclpy.ok = lambda: False
    rclpy.shutdown = lambda: None
    rclpy.spin = lambda *args, **kwargs: None
    rclpy.spin_once = lambda *args, **kwargs: None
    rclpy.spin_until_future_complete = lambda *args, **kwargs: None
    rclpy.get_default_context = lambda: types.SimpleNamespace(
        on_shutdown=lambda *args, **kwargs: None,
    )
    rclpy.node.Node = Node
    rclpy.action.ActionClient = _ActionEndpoint
    rclpy.action.ActionServer = _ActionEndpoint
    rclpy.action.CancelResponse = CancelResponse
    rclpy.callback_groups.MutuallyExclusiveCallbackGroup = _Group
    rclpy.callback_groups.ReentrantCallbackGroup = _Group
    rclpy.executors.MultiThreadedExecutor = MultiThreadedExecutor
    rclpy.qos.DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL=1)
    rclpy.qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    rclpy.qos.QoSHistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    rclpy.qos.QoSProfile = QoSProfile
    rclpy.qos.ReliabilityPolicy = types.SimpleNamespace(
        RELIABLE=_QoSValue(1), BEST_EFFORT=_QoSValue(2),
    )

    # std_msgs.msg ----------------------------------------------------
    std_msgs = _module('std_msgs')
    std_msgs.msg = _module('std_msgs.msg')

    class String(_BaseMsg):
        def __init__(self, data=''):
            super().__init__(data=data)

    std_msgs.msg.String = String

    # topological_navigation_msgs ------------------------------------
    topo_msgs = _module('topological_navigation_msgs')
    topo_msgs.action = _module('topological_navigation_msgs.action')
    topo_msgs.msg = _module('topological_navigation_msgs.msg')

    class GotoNode:
        class Feedback(_BaseMsg):
            pass

        class Result(_BaseMsg):
            def __init__(self, success=False):
                super().__init__(success=success)

    class ExecutePolicyMode:
        class Feedback(_BaseMsg):
            pass

        class Result(_BaseMsg):
            def __init__(self, success=False):
                super().__init__(success=success)

    class ClosestEdges(_BaseMsg):
        def __init__(self):
            super().__init__(edge_ids=[], distances=[])

    class NavStatistics(_BaseMsg):
        pass

    class TopologicalRoute(_BaseMsg):
        def __init__(self):
            super().__init__(nodes=[], edge_id=[])

    topo_msgs.action.GotoNode = GotoNode
    topo_msgs.action.ExecutePolicyMode = ExecutePolicyMode
    topo_msgs.msg.ClosestEdges = ClosestEdges
    topo_msgs.msg.NavStatistics = NavStatistics
    topo_msgs.msg.TopologicalRoute = TopologicalRoute

    # tf2_ros -----------------------------------------------------------
    tf2_ros = _module('tf2_ros')
    tf2_ros.buffer = _module('tf2_ros.buffer')
    tf2_ros.transform_listener = _module('tf2_ros.transform_listener')

    class TransformException(Exception):
        pass

    class Buffer:
        def __init__(self, *args, **kwargs):
            pass

    class TransformListener:
        def __init__(self, *args, **kwargs):
            pass

    tf2_ros.TransformException = TransformException
    tf2_ros.buffer.Buffer = Buffer
    tf2_ros.transform_listener.TransformListener = TransformListener


try:
    import rclpy  # noqa: F401
except ModuleNotFoundError:
    _install_ros_test_stubs()

try:
    import numpy as _np
    if not hasattr(_np, 'float_'):
        _np.float_ = _np.float64
    if not hasattr(_np, 'int'):
        _np.int = int
    if not hasattr(_np, 'int_'):
        _np.int_ = _np.int64
    if not hasattr(_np, 'complex'):
        _np.complex = complex
except Exception:
    pass

try:
    import scipy.special  # noqa: F401
except Exception:
    scipy = sys.modules.get('scipy')
    if scipy is None:
        scipy = types.ModuleType('scipy')
        sys.modules['scipy'] = scipy
    scipy.special = types.ModuleType('scipy.special')
    scipy.special.zeta = lambda *_args, **_kwargs: 0.0
    sys.modules['scipy.special'] = scipy.special

try:
    import scipy.spatial  # noqa: F401
except Exception:
    scipy = sys.modules.get('scipy')
    if scipy is None:
        scipy = types.ModuleType('scipy')
        sys.modules['scipy'] = scipy
    scipy.spatial = types.ModuleType('scipy.spatial')

    class _KDTree:
        def __init__(self, data):
            import numpy as np
            self.data = np.asarray(data, dtype=float)

        def query(self, points, k=1):
            import numpy as np
            query_points = np.asarray(points, dtype=float)
            if query_points.ndim == 1:
                query_points = query_points.reshape(1, -1)

            all_distances = []
            all_indices = []
            for point in query_points:
                distances = np.linalg.norm(self.data - point, axis=1)
                indices = np.argsort(distances)[:k]
                all_distances.append(distances[indices])
                all_indices.append(indices)

            all_distances = np.asarray(all_distances)
            all_indices = np.asarray(all_indices)
            if k == 1:
                return all_distances[:, 0], all_indices[:, 0]
            return all_distances, all_indices

    scipy.spatial.KDTree = _KDTree
    scipy.spatial.cKDTree = _KDTree
    sys.modules['scipy.spatial'] = scipy.spatial
