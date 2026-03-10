import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    graph_file = LaunchConfiguration("graph_file")
    behavior_map_file = LaunchConfiguration("behavior_map_file")
    map_file = LaunchConfiguration("map_file")
    params_file = LaunchConfiguration("params_file")
    use_rviz = LaunchConfiguration("use_rviz")
    enable_random_navigation = LaunchConfiguration("enable_random_navigation")
    random_nav_period = LaunchConfiguration("random_nav_period")

    gazebo_ros_pkg = get_package_share_directory("gazebo_ros")
    tb3_gazebo_pkg = get_package_share_directory("turtlebot3_gazebo")
    tb3_world = os.path.join(tb3_gazebo_pkg, "worlds", "turtlebot3_world.world")

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": tb3_world}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, "launch", "gzclient.launch.py")
        )
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_pkg, "launch", "robot_state_publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": "True"}.items(),
    )

    spawn_waiter = Node(
        package="topological_navigation",
        executable="gazebo_spawn_waiter",
        name="gazebo_spawn_waiter",
        output="screen",
        parameters=[
            {
                "entity_name": "waffle",
                "x": -2.0,
                "y": -0.5,
                "z": 0.01,
                "retry_period_sec": 2.0,
            }
        ],
    )

    delayed_spawn_turtlebot = TimerAction(period=6.0, actions=[spawn_waiter])

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"])
        ),
        launch_arguments={
            "slam": "False",
            "use_sim_time": "True",
            "autostart": "True",
            "map": map_file,
            "params_file": params_file,
            "graph": graph_file,
        }.items(),
    )

    nav2_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "rviz_launch.py"])
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "use_sim_time": "True",
            "rviz_config": PathJoinSubstitution(
                [FindPackageShare("topological_navigation"), "config", "route_map_view.rviz"]
            ),
        }.items(),
    )

    delayed_nav2_stack = TimerAction(
        period=14.0,
        actions=[nav2_bringup, nav2_rviz],
    )

    route_server = Node(
        package="nav2_route",
        executable="route_server",
        name="route_server",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": True,
                "graph_filepath": graph_file,
                "route_frame": "map",
                "base_frame": "base_link",
            },
        ],
    )

    route_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_route",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "autostart": True,
                "node_names": ["route_server"],
                "bond_timeout": 0.0,
            }
        ],
    )

    delayed_route_stack = TimerAction(
        period=18.0,
        actions=[route_server, route_lifecycle_manager],
    )

    wrapper = Node(
        package="topological_navigation",
        executable="route_bt_wrapper",
        name="route_bt_wrapper",
        output="screen",
        parameters=[
            {
                "graph_file": graph_file,
                "behavior_map_file": behavior_map_file,
                "route_frame": "map",
                "randomize_edge_behaviors": True,
                "edge_behavior_seed": 42,
                "default_controller_id": "FollowPath",
                "slow_controller_id": "SlowFollowPath",
                "reverse_controller_id": "ReverseFollowPath",
                "controller_selector_topic": "/controller_selector",
            }
        ],
    )

    interactive_node_markers = Node(
        package="topological_navigation",
        executable="interactive_node_markers",
        name="interactive_node_markers",
        output="screen",
        parameters=[
            {
                "graph_file": graph_file,
                "frame_id": "map",
                "named_route_action": "/execute_named_waypoints",
                "closest_node_topic": "/closest_node",
            }
        ],
    )

    closest_node_publisher = Node(
        package="topological_navigation",
        executable="closest_node_publisher",
        name="closest_node_publisher",
        output="screen",
        parameters=[
            {
                "graph_file": graph_file,
                "pose_cov_topic": "/amcl_pose",
                "closest_node_topic": "/closest_node",
            }
        ],
    )

    initial_pose_publisher = Node(
        package="topological_navigation",
        executable="initial_pose_publisher",
        name="initial_pose_publisher",
        output="screen",
        parameters=[
            {
                "x": -2.0,
                "y": -0.5,
                "yaw": 0.0,
                # Keep publishing long enough to catch AMCL after robot spawn/odom are live.
                "publish_count": 40,
                "publish_interval_sec": 1.0,
            }
        ],
    )

    delayed_initial_pose = TimerAction(
        period=52.0,
        actions=[initial_pose_publisher],
    )

    random_target_navigator = Node(
        package="topological_navigation",
        executable="random_target_navigator",
        name="random_target_navigator",
        output="screen",
        condition=IfCondition(enable_random_navigation),
        parameters=[
            {
                "graph_file": graph_file,
                "named_route_action": "/execute_named_waypoints",
                "closest_node_topic": "/closest_node",
                "period_sec": random_nav_period,
                "execute_navigation": True,
            }
        ],
    )

    delayed_random_navigation = TimerAction(
        period=80.0,
        actions=[random_target_navigator],
    )

    delayed_wrapper_stack = TimerAction(
        period=28.0,
        actions=[wrapper, closest_node_publisher, interactive_node_markers],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "graph_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("topological_navigation"), "config", "turtlebot3_graph.geojson"]
                ),
            ),
            DeclareLaunchArgument(
                "behavior_map_file",
                default_value="",
            ),
            DeclareLaunchArgument(
                "map_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("nav2_bringup"), "maps", "turtlebot3_world.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("topological_navigation"), "config", "nav2_params_custom.yaml"]
                ),
            ),
            DeclareLaunchArgument("use_rviz", default_value="True"),
            DeclareLaunchArgument("enable_random_navigation", default_value="False"),
            DeclareLaunchArgument("random_nav_period", default_value="30.0"),
            gzserver,
            gzclient,
            robot_state_publisher,
            delayed_spawn_turtlebot,
            delayed_nav2_stack,
            delayed_route_stack,
            delayed_wrapper_stack,
            delayed_initial_pose,
            delayed_random_navigation,
        ]
    )
