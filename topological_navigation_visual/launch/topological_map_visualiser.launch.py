#!/usr/bin/env python3
"""Launch file for the unified topological map visualiser.

Usage
-----
Live visualisation (subscribes to /topological_map_2)::

    ros2 launch topological_navigation_visual topological_map_visualiser.launch.py

Interactive editing from file::

    ros2 launch topological_navigation_visual topological_map_visualiser.launch.py \
        map_file:=/path/to/map.tmap2.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file', default_value='',
            description='Path to a .tmap2.yaml file. '
                        'Leave empty to subscribe to /topological_map_2.',
        ),
        DeclareLaunchArgument(
            'auto_save', default_value='false',
            description='Periodically save the map (every 30 s).',
        ),
        DeclareLaunchArgument(
            'marker_scale', default_value='0.5',
            description='Base scale factor for markers in RViz.',
        ),
        DeclareLaunchArgument(
            'edit_mode', default_value='true',
            description='Enable interactive drag-and-drop editing.',
        ),
        DeclareLaunchArgument(
            'nav_action_name', default_value='/topological_navigation',
            description='GotoNode action server name for click-to-navigate.',
        ),
        DeclareLaunchArgument(
            'show_node_labels', default_value='true',
            description='Render per-node text labels (disable for large '
                        'maps to keep RViz responsive).',
        ),
        DeclareLaunchArgument(
            'show_zones', default_value='true',
            description='Render node influence-zone polygons.',
        ),
        DeclareLaunchArgument(
            'show_edges', default_value='true',
            description='Render edges between nodes.',
        ),
        DeclareLaunchArgument(
            'auto_marker_scale', default_value='false',
            description='Derive marker_scale automatically from the spatial '
                        'spread of the map.',
        ),
        DeclareLaunchArgument(
            'interactive_marker_limit', default_value='750',
            description='Max node count for which interactive (editable) '
                        'markers are created; above this the map is shown '
                        'read-only.',
        ),
        Node(
            package='topological_navigation_visual',
            executable='topological_map_visualiser.py',
            name='topological_map_visualiser',
            output='screen',
            parameters=[{
                'map_file': LaunchConfiguration('map_file'),
                'auto_save': LaunchConfiguration('auto_save'),
                'marker_scale': LaunchConfiguration('marker_scale'),
                'edit_mode': LaunchConfiguration('edit_mode'),
                'nav_action_name': LaunchConfiguration('nav_action_name'),
                'show_node_labels': LaunchConfiguration('show_node_labels'),
                'show_zones': LaunchConfiguration('show_zones'),
                'show_edges': LaunchConfiguration('show_edges'),
                'auto_marker_scale': LaunchConfiguration('auto_marker_scale'),
                'interactive_marker_limit': LaunchConfiguration(
                    'interactive_marker_limit'),
            }],
        ),
    ])
