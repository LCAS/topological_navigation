'''
Author: Ibrahim Hroob <ihroob@lincoln.ac.uk> 2024
'''
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare the launch arguments
        DeclareLaunchArgument('tmap', default_value='test.yaml'),
        DeclareLaunchArgument('tmap_dir', default_value='/home/ros/tmap'),
        DeclareLaunchArgument('node_thresh', default_value='0.5'),
        DeclareLaunchArgument('lock_btn', default_value='6'),
        DeclareLaunchArgument('add_btn', default_value='1'),
        DeclareLaunchArgument('remove_btn', default_value='2'),
        DeclareLaunchArgument('gen_map_btn', default_value='3'),
        DeclareLaunchArgument('topic_joy', default_value='/joy'),
        DeclareLaunchArgument('topic_pose', default_value='/gps_base/odometry'),

        # Launch the manual_topomapping node
        Node(
            package='topological_navigation',
            executable='manual_topomapping.py',
            name='manual_topomapping',
            output='screen',
            parameters=[{
                'tmap'       : LaunchConfiguration('tmap'),
                'tmap_dir'  : LaunchConfiguration('tmap_dir'),
                'site_name'  : LaunchConfiguration('tmap'),
                'node_thresh': LaunchConfiguration('node_thresh'),
                'lock_btn'   : LaunchConfiguration('lock_btn'),
                'add_btn'    : LaunchConfiguration('add_btn'),
                'remove_btn' : LaunchConfiguration('remove_btn'),
                'gen_map_btn': LaunchConfiguration('gen_map_btn'),
                'topic_joy'  : LaunchConfiguration('topic_joy'),
                'topic_pose' : LaunchConfiguration('topic_pose'),
            }]
        ),
    ])
