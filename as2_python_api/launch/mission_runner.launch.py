#!/bin/python3

"""Mission runner launcher."""

from __future__ import annotations

# Copyright 2022 Universidad Politécnica de Madrid
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
#    * Neither the name of the the copyright holder nor the names of its
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


__authors__ = 'Pedro Arias Pérez'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

# import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
# from as2_core.declare_launch_arguments_from_config_file import \
# DeclareLaunchArgumentsFromConfigFile
# from as2_core.launch_configuration_from_config_file import LaunchConfigurationFromConfigFile
# from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # package_folder = get_package_share_directory(
    #     'gates_to_waypoints')
    # config_file = os.path.join(package_folder, 'config/config.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('log_level',
                              description='Logging level',
                              default_value='info'),
        DeclareLaunchArgument('use_sim_time',
                              description='Use simulation clock if true',
                              default_value='false'),
        DeclareLaunchArgument('namespace',
                              description='Drone namespace',
                              default_value=EnvironmentVariable(
                                  'AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgument('spin_rate',
                              description='Spin rate',
                              default_value='20'),
        DeclareLaunchArgument('mission_path',
                              description='Path where the mission is located'),
        DeclareLaunchArgument('mission_name',
                              description='Name of the mission to be executed'),
        # DeclareLaunchArgumentsFromConfigFile('config_file',
        #                                      config_file,
        #                                      description='Configuration file'),
        Node(
            package='as2_python_api',
            namespace=LaunchConfiguration('namespace'),
            executable='mission_runner',
            output='screen',
            arguments=['--use_sim_time', LaunchConfiguration('use_sim_time'),
                       '--ns', LaunchConfiguration('namespace'),
                       '--spin-rate', LaunchConfiguration('spin_rate'),
                       '--log-level', LaunchConfiguration('log_level')],
            emulate_tty=True,
            parameters=[
                {
                    'mission_path': LaunchConfiguration('mission_path'),
                    'mission_name': LaunchConfiguration('mission_name'),
                },
                # LaunchConfigurationFromConfigFile(
                #     'config_file',
                #     default_file=config_file)
            ]
        ),
    ])
