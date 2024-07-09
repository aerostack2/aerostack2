#!/usr/bin/env python3

# Copyright 2024 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Launch the AS2 platform behaviors test."""

__authors__ = 'CVAR'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_takeoff = PathJoinSubstitution([
        FindPackageShare('takeoff_behavior'),
        'config', 'takeoff_behavior.yaml'
    ])
    config_land = PathJoinSubstitution([
        FindPackageShare('land_behavior'),
        'config', 'land_behavior.yaml'
    ])
    config_go_to = PathJoinSubstitution([
        FindPackageShare('go_to_behavior'),
        'config', 'go_to_behavior.yaml'
    ])
    config_follow_path = PathJoinSubstitution([
        FindPackageShare('follow_path_behavior'),
        'config', 'follow_path_behavior.yaml'
    ])

    takeoff = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([
            FindPackageShare('takeoff_behavior'),
            'launch', 'takeoff_behavior_launch.py'
        ])]),
        launch_arguments={
            'drone_id': LaunchConfiguration('drone_id'),
            'config': LaunchConfiguration('config_takeoff')
        }.items(),
    )

    land = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([
            FindPackageShare('land_behavior'),
            'launch', 'land_behavior_launch.py'
        ])]),
        launch_arguments={
            'drone_id': LaunchConfiguration('drone_id'),
            'config': LaunchConfiguration('config_land')
        }.items(),
    )

    go_to = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([
            FindPackageShare('go_to_behavior'),
            'launch', 'go_to_behavior_launch.py'
        ])]),
        launch_arguments={
            'drone_id': LaunchConfiguration('drone_id'),
            'config': LaunchConfiguration('config_go_to')
        }.items(),
    )

    follow_path = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([
            FindPackageShare('follow_path_behavior'),
            'launch', 'follow_path_behavior_launch.py'
        ])]),
        launch_arguments={
            'drone_id': LaunchConfiguration('drone_id'),
            'config': LaunchConfiguration('config_follow_path')
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value=EnvironmentVariable(
            'AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgument('config_takeoff', default_value=config_takeoff),
        DeclareLaunchArgument('config_land', default_value=config_land),
        DeclareLaunchArgument('config_go_to', default_value=config_go_to),
        DeclareLaunchArgument('config_follow_path',
                              default_value=config_follow_path),
        takeoff,
        land,
        go_to,
        follow_path
    ])
