#!/usr/bin/env python3

# Copyright 2023 Universidad Politécnica de Madrid
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
"""Launch Pixhawk platform node."""

__authors__ = 'Miguel Fernández Cortizas, Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import os

from ament_index_python.packages import get_package_share_directory
from as2_core.declare_launch_arguments_from_config_file import DeclareLaunchArgumentsFromConfigFile
from as2_core.launch_configuration_from_config_file import LaunchConfigurationFromConfigFile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_platform_config_file():
    """Return the platform config file."""
    package_folder = get_package_share_directory('as2_platform_mavlink')
    return os.path.join(package_folder,
                        'config/platform_config_file.yaml')


def get_control_modes_file():
    """Return the control modes file."""
    package_folder = get_package_share_directory('as2_platform_mavlink')
    return os.path.join(package_folder,
                        'config/control_modes.yaml')


def generate_launch_description():
    """Entrypoint."""
    return LaunchDescription([
        DeclareLaunchArgument('namespace',
                              default_value='drone0',
                              description='Drone namespace'),
        DeclareLaunchArgument('control_modes_file',
                              default_value=get_control_modes_file(),
                              description='Platform control modes file'),
        DeclareLaunchArgumentsFromConfigFile(name='platform_config_file',
                                             source_file=get_platform_config_file(),
                                             description='Platform configuration file'),
        Node(
            package='as2_platform_mavlink',
            executable='as2_platform_mavlink_node',
            name='platform',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'control_modes_file': LaunchConfiguration('control_modes_file'),
                },
                LaunchConfigurationFromConfigFile('platform_config_file',
                                                  default_file=get_platform_config_file())
            ]
        ),
    ])
