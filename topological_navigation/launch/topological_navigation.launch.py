#!/usr/bin/env python3
# Copyright (c) 2026, topological_navigation contributors
# Licensed under the MIT License.
"""Launch the full topological navigation stack with simulation.

Launches (in order):
    1. Map Manager -- loads and publishes the topological map
    2. Localisation -- determines current/closest node
    3. Fake Nav2 simulator -- virtual robot + fake action servers
    4. Topological map visualiser -- RViz markers
    5. RViz -- with a pre-configured display layout
    6. Topological web app -- browser UI on http://localhost:8083/

Usage
-----
Default (mixed_actions_map)::

    ros2 launch topological_navigation topological_navigation.launch.py

Custom map file::

    ros2 launch topological_navigation topological_navigation.launch.py \
        map_path:=/absolute/path/to/my_map.tmap2.yaml

Split map and navigation config files::

    ros2 launch topological_navigation topological_navigation.launch.py \
        map_path:=/absolute/path/to/my_map.tmap2.yaml \
        navigation_config_file:=/absolute/path/to/topological_navigation_config.yaml

Disable RViz and/or the fake Nav2 server::

    ros2 launch topological_navigation topological_navigation.launch.py \
        rviz:=false fake_nav2:=false

Disable the web app or change its port::

    ros2 launch topological_navigation topological_navigation.launch.py \
        web_app:=false
    ros2 launch topological_navigation topological_navigation.launch.py \
        web_app_port:=8090

Configure the simulated robot speed and other fake Nav2 parameters::

    ros2 launch topological_navigation topological_navigation.launch.py \
        robot_speed:=1.2 angular_speed:=2.0 goal_tolerance:=0.1

Nodes are started in sequence using event handlers (triggered when the
previous node's process starts) rather than fixed timers, so the stack
comes up as soon as each prerequisite node is actually running.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    topo_nav_share = get_package_share_directory('topological_navigation')

    # --- Default paths ---
    default_map = os.path.join(
        topo_nav_share, 'config', 'riseholme.tmap3.yaml',
    )
    default_rviz = os.path.join(
        topo_nav_share, 'rviz', 'topological_navigation.rviz',
    )

    default_navigation_config_file = os.path.join(
        topo_nav_share, 'config', 'topological_navigation_config.yaml',
    )

    # =================================================================
    # Launch arguments
    # =================================================================
    launch_args = [
        DeclareLaunchArgument(
            'map_path', default_value=default_map,
            description='Absolute path to a .tmap2.yaml topological map',
        ),
        DeclareLaunchArgument(
            'navigation_config_file',
            default_value=default_navigation_config_file,
            description=(
                'Optional path to topological_navigation_config.yaml containing '
                'the actions and definitions sections'
            ),
        ),
        DeclareLaunchArgument(
            'rviz_config', default_value=default_rviz,
            description='Path to the RViz config file',
        ),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch RViz (true/false)',
        ),
        DeclareLaunchArgument(
            'fake_nav2', default_value='true',
            description='Launch the fake Nav2 simulator server (true/false)',
        ),
        # --- Fake Nav2 simulator parameters (passed through to
        # topological_nav_simulator/fake_nav2_server) ---
        DeclareLaunchArgument(
            'robot_speed', default_value='1.5',
            description='Simulated robot linear speed (m/s)',
        ),
        DeclareLaunchArgument(
            'angular_speed', default_value='4.0',
            description='Simulated robot angular speed (rad/s)',
        ),
        DeclareLaunchArgument(
            'goal_tolerance', default_value='0.15',
            description='Distance tolerance for reaching goals (m)',
        ),
        DeclareLaunchArgument(
            'update_rate', default_value='30.0',
            description='TF and marker publish rate (Hz)',
        ),
        DeclareLaunchArgument(
            'web_app', default_value='true',
            description='Launch the topological navigation web app (true/false)',
        ),
        DeclareLaunchArgument(
            'web_app_port', default_value='8083',
            description='HTTP port of the topological navigation web app',
        ),
        DeclareLaunchArgument(
            'initial_x', default_value='25.75',
            description='Initial robot X position',
        ),
        DeclareLaunchArgument(
            'initial_y', default_value='2.8',
            description='Initial robot Y position',
        ),
        DeclareLaunchArgument(
            'initial_yaw', default_value='0.0',
            description='Initial robot yaw (radians)',
        ),
        DeclareLaunchArgument(
            'map_frame', default_value='map',
            description='Map TF frame',
        ),
        DeclareLaunchArgument(
            'odom_frame', default_value='odom',
            description='Odometry TF frame',
        ),
        DeclareLaunchArgument(
            'base_frame', default_value='base_link',
            description='Robot base TF frame',
        ),
    ]

    # =================================================================
    # 1. Map Manager
    # =================================================================
    map_manager_node = Node(
        package='topological_navigation',
        executable='map_manager2.py',
        name='topological_map_manager_2',
        output='screen',
        arguments=[
            LaunchConfiguration('map_path'),
            '--navigation-config-file',
            LaunchConfiguration('navigation_config_file'),
        ],
    )

    # =================================================================
    # 2. Localisation
    # =================================================================
    localisation_node = Node(
        package='topological_navigation',
        executable='localisation2.py',
        name='topological_localisation',
        output='screen',
    )

    # =================================================================
    # 3. Fake Nav2 simulator
    # =================================================================
    fake_nav2_node = Node(
        package='topological_nav_simulator',
        executable='fake_nav2_server',
        name='fake_nav2_server',
        output='screen',
        condition=IfCondition(LaunchConfiguration('fake_nav2')),
        parameters=[{
            'robot_speed': LaunchConfiguration('robot_speed'),
            'angular_speed': LaunchConfiguration('angular_speed'),
            'goal_tolerance': LaunchConfiguration('goal_tolerance'),
            'update_rate': LaunchConfiguration('update_rate'),
            'initial_x': LaunchConfiguration('initial_x'),
            'initial_y': LaunchConfiguration('initial_y'),
            'initial_yaw': LaunchConfiguration('initial_yaw'),
            'map_frame': LaunchConfiguration('map_frame'),
            'odom_frame': LaunchConfiguration('odom_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
        }],
    )

    # =================================================================
    # 4. Topological navigation server
    # =================================================================
    navigation_node = Node(
        package='topological_navigation',
        executable='navigation2.py',
        name='topological_navigation',
        output='screen',
        parameters=[{
            'coarse_white_extension_m': 4.0,
            'route_white_extension_m': 2.0,
            'metric_map_resolution': 1.0,
            'route_segment_resolution': 0.05,
            'route_segment_border_width': 0.25,
            'route_segment_padding': 0.0,
        }],
    )

    # =================================================================
    # 5. Topological map visualiser
    # =================================================================
    visualiser_node = Node(
        package='topological_navigation_visual',
        executable='topological_map_visualiser.py',
        name='topological_map_visualiser',
        output='screen',
        parameters=[{
            'edit_mode': True,
        }],
    )

    # =================================================================
    # 6. RViz
    # =================================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=[
            '-d', LaunchConfiguration('rviz_config'),
        ],
    )

    # =================================================================
    # 7. Topological web app
    # =================================================================
    web_app_node = Node(
        package='topological_navigation_visual',
        executable='topological_web_app.py',
        name='topological_web_app',
        output='screen',
        condition=IfCondition(LaunchConfiguration('web_app')),
        parameters=[{
            'port': ParameterValue(
                LaunchConfiguration('web_app_port'), value_type=int,
            ),
        }],
    )

    # =================================================================
    # Sequencing: start each node once the previous one has actually
    # started its process, instead of relying on fixed-duration timers.
    # =================================================================
    event_handlers = [
        RegisterEventHandler(
            OnProcessStart(
                target_action=map_manager_node,
                on_start=[localisation_node, fake_nav2_node],
            ),
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=localisation_node,
                on_start=[navigation_node],
            ),
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=navigation_node,
                on_start=[visualiser_node],
            ),
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=visualiser_node,
                on_start=[rviz_node, web_app_node],
            ),
        ),
    ]

    return LaunchDescription(
        launch_args + [map_manager_node] + event_handlers,
    )
