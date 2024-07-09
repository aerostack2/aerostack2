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

"""Launch the AS2 platform behaviors."""

__authors__ = 'CVAR'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the AS2 platform behaviors."""
    return LaunchDescription([
        DeclareLaunchArgument('namespace'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='as2_behaviors_platform',
            executable='arm_behavior',
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen',
            emulate_tty=True
        ),

        Node(
            package='as2_behaviors_platform',
            executable='offboard_behavior',
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen',
            emulate_tty=True
        ),

        DeclareLaunchArgument('follow_path_plugin_name'),
        DeclareLaunchArgument('follow_path_speed', default_value='0.5'),
        DeclareLaunchArgument('follow_path_threshold', default_value='0.3'),
        Node(
            package='follow_path_behavior',
            executable='follow_path_behavior_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                {'namespace': LaunchConfiguration('namespace')},
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'plugin_name': LaunchConfiguration(
                    'follow_path_plugin_name')},
                {'follow_path_speed': LaunchConfiguration(
                    'follow_path_speed')},
                {'follow_path_threshold': LaunchConfiguration(
                    'follow_path_threshold')}
            ],
            output='screen',
            emulate_tty=True
        ),

        DeclareLaunchArgument('go_to_plugin_name'),
        DeclareLaunchArgument('go_to_speed', default_value='0.5'),
        DeclareLaunchArgument('go_to_threshold', default_value='0.3'),
        Node(
            package='go_to_behavior',
            executable='go_to_behavior_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                {'namespace': LaunchConfiguration('namespace')},
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'plugin_name': LaunchConfiguration('go_to_plugin_name')},
                {'go_to_speed': LaunchConfiguration(
                    'go_to_speed')},
                {'go_to_threshold': LaunchConfiguration('go_to_threshold')}
            ],
            output='screen',
            emulate_tty=True
        ),

        DeclareLaunchArgument('land_plugin_name'),
        DeclareLaunchArgument('land_speed', default_value='0.2'),
        Node(
            package='land_behavior',
            executable='land_behavior_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                {'namespace': LaunchConfiguration('namespace')},
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'plugin_name': LaunchConfiguration('land_plugin_name')},
                {'land_speed': LaunchConfiguration(
                    'land_speed')}
            ],
            output='screen',
            emulate_tty=True
        ),

        DeclareLaunchArgument('takeoff_plugin_name'),
        DeclareLaunchArgument('takeoff_height', default_value='1.0'),
        DeclareLaunchArgument('takeoff_speed', default_value='0.5'),
        DeclareLaunchArgument('takeoff_threshold', default_value='0.3'),
        Node(
            package='takeoff_behavior',
            executable='takeoff_behavior_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                {'namespace': LaunchConfiguration('namespace')},
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'plugin_name': LaunchConfiguration('takeoff_plugin_name')},
                {'takeoff_height': LaunchConfiguration(
                    'takeoff_height')},
                {'takeoff_speed': LaunchConfiguration(
                    'takeoff_speed')},
                {'takeoff_threshold': LaunchConfiguration('takeoff_threshold')}
            ],
            output='screen',
            emulate_tty=True
        ),
    ])
