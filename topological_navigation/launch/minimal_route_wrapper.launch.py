from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    graph_file = LaunchConfiguration("graph_file")
    behavior_map_file = LaunchConfiguration("behavior_map_file")

    route_params = {
        "use_sim_time": False,
        "base_frame": "base_link",
        "route_frame": "map",
        "graph_filepath": graph_file,
        "graph_file_loader": "GeoJsonGraphFileLoader",
        "GeoJsonGraphFileLoader.plugin": "nav2_route::GeoJsonGraphFileLoader",
        "edge_cost_functions": ["DistanceScorer"],
        "DistanceScorer.plugin": "nav2_route::DistanceScorer",
        "operations": [],
        "enable_nn_search": False,
        "tracker_update_rate": 10.0,
    }

    route_server = Node(
        package="nav2_route",
        executable="route_server",
        name="route_server",
        output="screen",
        parameters=[route_params],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_route",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "autostart": True,
                "node_names": ["route_server"],
            }
        ],
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
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "graph_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("topological_navigation"), "config", "minimal_graph.geojson"]
                ),
            ),
            DeclareLaunchArgument(
                "behavior_map_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("topological_navigation"), "config", "edge_behaviors.yaml"]
                ),
            ),
            route_server,
            lifecycle_manager,
            wrapper,
            closest_node_publisher,
        ]
    )
