# -*- coding: utf-8 -*-
#! /usr/bin/env python3
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import os
from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    LD = LaunchDescription()

    ## Define Arguments
    tmap_input = LaunchConfiguration('tmap')
    tmap_default = os.getenv('TMAP_FILE')
    LD.add_action(DeclareLaunchArgument('tmap', default_value=tmap_default))

    ## Topological Map Server
    LD.add_action(Node(
        package='topological_navigation',
        executable='map_manager2.py',
        name='topomap2_server',
        arguments=[tmap_default]
    ))
    LD.add_action(Node(
        package='topological_navigation',
        executable='topological_transform_publisher.py',
        name='topological_transform_publisher'
    ))
    LD.add_action(Node(
        package='topological_navigation',
        executable='topomap_marker2.py',
        name='topomap_marker2'
    ))

    ## Restrictions Handler Example
    LD.add_action(Node(
        package='topological_navigation',
        executable='restrictions_handler.py',
        name='restrictions_handler',
        parameters=[{'enable_eval_sub': True},
                    {'initial_restriction': "'robot_short' in '$' or '$' == 'True'"}],
        remappings=[('/topological_map_2', '/topological_map_2')]
    ))
    LD.add_action(Node(
        package='topological_navigation',
        executable='topological_transform_publisher.py',
        name='restricted_topological_transform_publisher',
        remappings=[('/topological_map_2', '/restrictions_handler/topological_map_2')]
    ))
    LD.add_action(Node(
        package='topological_navigation',
        executable='topomap_marker2.py',
        name='restricted_topomap_marker2',
        remappings=[('/topological_map_2', '/restrictions_handler/topological_map_2')]
    ))


    ## Execute all Components
    return LD
