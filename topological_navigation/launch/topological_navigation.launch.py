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
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    topo_nav_share = get_package_share_directory('topological_navigation')

    # --- Default paths ---
    default_map = os.path.join(
        topo_nav_share, 'config', 'mixed_actions_map.yaml',
    )
    default_rviz = os.path.join(
        topo_nav_share, 'rviz', 'topological_navigation.rviz',
    )

    return LaunchDescription([
        # =============================================================
        # Launch arguments
        # =============================================================
        DeclareLaunchArgument(
            'map_path', default_value=default_map,
            description='Absolute path to a .tmap2.yaml topological map',
        ),
        DeclareLaunchArgument(
            'navigation_config_file',
            default_value='',
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
        DeclareLaunchArgument(
            'web_app', default_value='true',
            description='Launch the topological navigation web app (true/false)',
        ),
        DeclareLaunchArgument(
            'web_app_port', default_value='8083',
            description='HTTP port of the topological navigation web app',
        ),
        DeclareLaunchArgument(
            'initial_x', default_value='0.0',
            description='Initial robot X position',
        ),
        DeclareLaunchArgument(
            'initial_y', default_value='0.0',
            description='Initial robot Y position',
        ),
        DeclareLaunchArgument(
            'initial_yaw', default_value='0.0',
            description='Initial robot yaw (radians)',
        ),

        # =============================================================
        # 1. Map Manager
        # =============================================================
        Node(
            package='topological_navigation',
            executable='map_manager2.py',
            name='topological_map_manager_2',
            output='screen',
            arguments=[
                LaunchConfiguration('map_path'),
                '--navigation-config-file',
                LaunchConfiguration('navigation_config_file'),
            ],
        ),

        # =============================================================
        # 2. Localisation (delayed 2 s so map is published first)
        # =============================================================
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='topological_navigation',
                    executable='localisation2.py',
                    name='topological_localisation',
                    output='screen',
                ),
            ],
        ),

        # =============================================================
        # 3. Fake Nav2 simulator (delayed 2 s)
        # =============================================================
        TimerAction(
            period=2.0,
            condition=IfCondition(LaunchConfiguration('fake_nav2')),
            actions=[
                Node(
                    package='topological_nav_simulator',
                    executable='fake_nav2_server',
                    name='fake_nav2_server',
                    output='screen',
                    parameters=[{
                        'initial_x': LaunchConfiguration('initial_x'),
                        'initial_y': LaunchConfiguration('initial_y'),
                        'initial_yaw': LaunchConfiguration('initial_yaw'),
                    }],
                ),
            ],
        ),

        # =============================================================
        # 4. Topological navigation server (delayed 3 s)
        # =============================================================
        TimerAction(
            period=3.0,
            actions=[
                Node(
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
                ),
            ],
        ),

        # =============================================================
        # 5. Topological map visualiser (delayed 4 s)
        # =============================================================
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='topological_navigation_visual',
                    executable='topological_map_visualiser.py',
                    name='topological_map_visualiser',
                    output='screen',
                    parameters=[{
                        'edit_mode': True,
                    }],
                ),
            ],
        ),

        # =============================================================
        # 6. RViz (delayed 5 s)
        # =============================================================
        TimerAction(
            period=5.0,
            condition=IfCondition(LaunchConfiguration('rviz')),
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    arguments=[
                        '-d', LaunchConfiguration('rviz_config'),
                    ],
                ),
            ],
        ),

        # =============================================================
        # 7. Topological web app (delayed 4 s)
        # =============================================================
        TimerAction(
            period=4.0,
            condition=IfCondition(LaunchConfiguration('web_app')),
            actions=[
                Node(
                    package='topological_navigation_visual',
                    executable='topological_web_app.py',
                    name='topological_web_app',
                    output='screen',
                    parameters=[{
                        'port': ParameterValue(
                            LaunchConfiguration('web_app_port'), value_type=int,
                        ),
                    }],
                ),
            ],
        ),
    ])
