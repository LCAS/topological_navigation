from pathlib import Path
import sys
from threading import Event
from threading import Thread
import launch
import launch_pytest
import launch_ros
import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

@launch_pytest.fixture
def generate_test_description():
    path_to_test = Path(__file__).parent
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            executable=sys.executable,
            arguments=[str(path_to_test / 'static_transformer.py')],
            additional_env={'PYTHONUNBUFFERED': '1'},
            name='static_transformer',
            output='screen',
        ),
        launch_ros.actions.Node(
            executable=sys.executable,
            arguments=[str(path_to_test / '..' / 'topological_navigation'/'scripts' /'map_manager2.py'), str(path_to_test / 'conf' / 'network_autogen.tmap2.yaml')],
            additional_env={'PYTHONUNBUFFERED': '1'},
            name='map_manager',
            output='screen',
        ),
        launch_ros.actions.Node(
            executable=sys.executable,
            arguments=[str(path_to_test / '..' / 'topological_navigation'/'scripts'/'topological_transform_publisher.py')],
            additional_env={'PYTHONUNBUFFERED': '1'},
            name='topological_transform_publisher',
            output='screen',
        ),
        launch_ros.actions.Node(
            executable=sys.executable,
            arguments=[str(path_to_test / '..' / 'topological_navigation'/ 'scripts'/'navigation2.py')],
            additional_env={'PYTHONUNBUFFERED': '1'},
            name='topological_navigation_core',
            output='screen',
        ),
        launch_ros.actions.Node(
            executable=sys.executable,
            arguments=[str(path_to_test / '..' / 'topological_navigation'/ 'scripts'/'localisation2.py')],
            additional_env={'PYTHONUNBUFFERED': '1'},
            name='topological_navigation_localisation',
            output='screen',
        ),
    ])


@pytest.mark.launch(fixture=generate_test_description)
def test_check_if_topological_map_is_published():
    rclpy.init()
    try:
        node = NavigationClient('test_node')
        node.initialize()
        msgs_received_flag = node.topmap_event_object.wait(timeout=20.0)
        assert msgs_received_flag, 'Did not receive topological map !'
    finally:
        rclpy.shutdown()

@pytest.mark.launch(fixture=generate_test_description)
def test_check_if_closest_node_is_published():
    rclpy.init()
    try:
        node = NavigationClient('test_node')
        node.initialize()
        msgs_received_flag = node.closest_node_event_object.wait(timeout=20.0)
        assert msgs_received_flag, 'Did not receive closest node info !'
    finally:
        rclpy.shutdown()

@pytest.mark.launch(fixture=generate_test_description)
def test_check_if_current_node_is_published():
    rclpy.init()
    try:
        node = NavigationClient('test_node')
        node.initialize()
        msgs_received_flag = node.current_node_event_object.wait(timeout=20.0)
        assert msgs_received_flag, 'Did not receive current node info !'
    finally:
        rclpy.shutdown()

class NavigationClient(Node):
    def __init__(self, name='test_node'):
        super().__init__(name)
        self.topmap_event_object = Event()
        self.closest_node_event_object = Event()
        self.current_node_event_object = Event()

    def initialize(self):
        self.topmap_sub = self.create_subscription(String, '/topological_map_2', self.topmap_sub_callback, 1)
        self.closest_node_sub = self.create_subscription(String, 'closest_node', self.closest_node_callback, 1)
        self.current_node_sub = self.create_subscription(String, 'current_node', self.current_node_callback, 1)

        self.ros_spin_thread = Thread(target=lambda node: rclpy.spin(node), args=(self,))
        self.ros_spin_thread.start()

    def topmap_sub_callback(self, data):
        self.topmap_event_object.set()
    
    def closest_node_callback(self, data):
        self.closest_node_event_object.set()

    def current_node_callback(self, data):
        self.current_node_event_object.set()
