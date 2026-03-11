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
    use_gzclient = LaunchConfiguration("use_gzclient")
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
        ),
        condition=IfCondition(use_gzclient),
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_pkg, "launch", "robot_state_publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": "True"}.items(),
    )

    # Spawn robot via the demo package's spawn waiter node.
    spawn_waiter = Node(
        package="topological_navigation_demo",
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

    # Fake localizer: reads Gazebo ground-truth and broadcasts map→odom TF.
    # Started early so the map frame exists before Nav2 activates.
    fake_localizer = Node(
        package="topological_navigation_demo",
        executable="fake_localizer",
        name="fake_localizer",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "robot_model_name": "waffle",
            "map_frame": "map",
            "odom_frame": "odom",
            "base_frame": "base_footprint",
        }],
    )

    # Start fake localizer early — before Nav2 — so its map→odom TF is live.
    delayed_fake_localizer = TimerAction(period=9.0, actions=[fake_localizer])

    # Nav2 navigation stack (no AMCL — map frame is provided by fake_localizer).
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": "True",
            "autostart": "True",
            "params_file": params_file,
        }.items(),
    )

    # Map server — serves the static occupancy grid for the global costmap.
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[params_file, {"yaml_filename": map_file, "use_sim_time": True}],
    )

    # Lifecycle manager for map_server only (AMCL excluded — using fake_localizer).
    lifecycle_manager_localization = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "autostart": True,
            "node_names": ["map_server"],
            "bond_timeout": 0.0,
        }],
    )

    nav2_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "rviz_launch.py"])
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "use_sim_time": "True",
            "rviz_config": PathJoinSubstitution(
                [FindPackageShare("topological_navigation_demo"), "config", "route_map_view.rviz"]
            ),
        }.items(),
    )

    # Nav2 starts at t=16s — after fake_localizer has had ~7s to begin broadcasting.
    delayed_nav2_stack = TimerAction(
        period=16.0,
        actions=[
            nav2_navigation,
            map_server,
            lifecycle_manager_localization,
            nav2_rviz,
        ],
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
        period=20.0,
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
                "slow_behavior_tree": "",
                "reverse_behavior_tree": "",
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
                "interactive_markers_namespace": "topological_nodes",
                "named_route_action": "/execute_named_waypoints",
                "closest_node_topic": "/closest_node",
                "pose_topic": "/robot_pose",
                "pose_cov_topic": "/amcl_pose",
                "odom_topic": "/odom",
                "selected_route_topic": "/selected_topological_route",
                "marker_z_offset": 0.05,
                "click_sphere_scale_factor": 0.8,
            }
        ],
    )

    edge_behavior_visualizer = Node(
        package="topological_navigation",
        executable="edge_behavior_visualizer",
        name="edge_behavior_visualizer",
        output="screen",
        parameters=[
            {
                "graph_file": graph_file,
                "frame_id": "map",
                "behavior_map_file": behavior_map_file,
                "randomize_edge_behaviors": True,
                "edge_behavior_seed": 42,
                "default_controller_id": "FollowPath",
                "slow_controller_id": "SlowFollowPath",
                "reverse_controller_id": "ReverseFollowPath",
                "marker_topic": "/topological_edges_colored",
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

    random_target_navigator = Node(
        package="topological_navigation_demo",
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
        period=30.0,
        actions=[wrapper, closest_node_publisher, interactive_node_markers, edge_behavior_visualizer],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "graph_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("topological_navigation_demo"), "config", "turtlebot3_graph.geojson"]
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
                    [FindPackageShare("topological_navigation_demo"), "config", "nav2_params_turtlebot3.yaml"]
                ),
            ),
            DeclareLaunchArgument("use_rviz", default_value="True"),
            DeclareLaunchArgument("use_gzclient", default_value="False"),
            DeclareLaunchArgument("enable_random_navigation", default_value="False"),
            DeclareLaunchArgument("random_nav_period", default_value="30.0"),
            gzserver,
            gzclient,
            robot_state_publisher,
            delayed_spawn_turtlebot,
            delayed_fake_localizer,
            delayed_nav2_stack,
            delayed_route_stack,
            delayed_wrapper_stack,
            delayed_random_navigation,
        ]
    )
