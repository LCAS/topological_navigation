from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Example deployment launch — demonstrates how a custom deployment package
    can override graph, params, behavior map, and RViz config while reusing
    the full Gazebo simulation infrastructure from topological_navigation_demo.

    To adapt this for your own robot/environment:
      1. Replace config/deployment_graph.geojson with your topological graph.
      2. Replace config/deployment_nav2_params.yaml with your Nav2 params
         (update map path, speeds, robot footprint, etc.).
      3. Edit config/deployment_navigate_to_pose.xml / ..._through_poses.xml
         with any custom BT logic you need.
      4. Optionally edit config/deployment_edge_behaviors.yaml to pin specific
         edge IDs to specific BT files.
      5. Update config/deployment_rviz.rviz for your preferred RViz layout.
    """

    deployment_pkg = FindPackageShare("topological_navigation_demo_deployment")

    graph_file = LaunchConfiguration("graph_file")
    params_file = LaunchConfiguration("params_file")
    behavior_map_file = LaunchConfiguration("behavior_map_file")
    use_rviz = LaunchConfiguration("use_rviz")
    use_gzclient = LaunchConfiguration("use_gzclient")
    enable_random_navigation = LaunchConfiguration("enable_random_navigation")
    random_nav_period = LaunchConfiguration("random_nav_period")

    # Include the demo sim launch, overriding graph/params/behavior_map/rviz
    # with this deployment's own configuration.
    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("topological_navigation_demo"), "launch", "turtlebot_route_compose.launch.py"]
            )
        ),
        launch_arguments={
            "graph_file": graph_file,
            "params_file": params_file,
            "behavior_map_file": behavior_map_file,
            "use_rviz": use_rviz,
            "use_gzclient": use_gzclient,
            "enable_random_navigation": enable_random_navigation,
            "random_nav_period": random_nav_period,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "graph_file",
                default_value=PathJoinSubstitution(
                    [deployment_pkg, "config", "deployment_graph.geojson"]
                ),
                description="Topological graph GeoJSON for this deployment.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [deployment_pkg, "config", "deployment_nav2_params.yaml"]
                ),
                description="Nav2 params for this deployment.",
            ),
            DeclareLaunchArgument(
                "behavior_map_file",
                default_value=PathJoinSubstitution(
                    [deployment_pkg, "config", "deployment_edge_behaviors.yaml"]
                ),
                description="Edge behavior map for this deployment.",
            ),
            DeclareLaunchArgument("use_rviz", default_value="True"),
            DeclareLaunchArgument("use_gzclient", default_value="False"),
            DeclareLaunchArgument("enable_random_navigation", default_value="False"),
            DeclareLaunchArgument("random_nav_period", default_value="30.0"),
            demo_launch,
        ]
    )
