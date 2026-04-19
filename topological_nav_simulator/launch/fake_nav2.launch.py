# Copyright (c) 2026, topological_navigation contributors
# Licensed under the MIT License.
"""Launch fake Nav2 server with virtual robot visualization."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for the fake Nav2 simulator."""
    return LaunchDescription([
        # --- Arguments ---
        DeclareLaunchArgument(
            'robot_speed', default_value='0.8',
            description='Simulated robot linear speed (m/s)'),
        DeclareLaunchArgument(
            'angular_speed', default_value='1.5',
            description='Simulated robot angular speed (rad/s)'),
        DeclareLaunchArgument(
            'goal_tolerance', default_value='0.15',
            description='Distance tolerance for reaching goals (m)'),
        DeclareLaunchArgument(
            'update_rate', default_value='30.0',
            description='TF and marker publish rate (Hz)'),
        DeclareLaunchArgument(
            'initial_x', default_value='0.0',
            description='Initial robot X position'),
        DeclareLaunchArgument(
            'initial_y', default_value='0.0',
            description='Initial robot Y position'),
        DeclareLaunchArgument(
            'initial_yaw', default_value='0.0',
            description='Initial robot yaw (radians)'),
        DeclareLaunchArgument(
            'map_frame', default_value='map',
            description='Map TF frame'),
        DeclareLaunchArgument(
            'odom_frame', default_value='odom',
            description='Odometry TF frame'),
        DeclareLaunchArgument(
            'base_frame', default_value='base_link',
            description='Robot base TF frame'),

        # --- Fake Nav2 Server ---
        Node(
            package='topological_nav_simulator',
            executable='fake_nav2_server',
            name='fake_nav2_server',
            output='screen',
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
        ),
    ])
