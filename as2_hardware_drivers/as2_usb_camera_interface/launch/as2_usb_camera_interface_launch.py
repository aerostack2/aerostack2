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

"""Launch file for the as2_usb_camera_interface package."""

__authors__ = 'David Pérez Saura, Miguel Fernandez Cortizas'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import os

from ament_index_python.packages import get_package_share_directory
from as2_core.declare_launch_arguments_from_config_file import DeclareLaunchArgumentsFromConfigFile
from as2_core.launch_configuration_from_config_file import LaunchConfigurationFromConfigFile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Entrypoint."""
    # Get default platform configuration file
    package_folder = get_package_share_directory(
        'as2_usb_camera_interface')

    config_file = os.path.join(package_folder,
                               'config/config_file_default.yaml')
    camera_calibration_file = os.path.join(package_folder,
                                           'config/camera_calibration_default.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('log_level',
                              description='Logging level',
                              default_value='info'),
        DeclareLaunchArgument('namespace',
                              description='Drone namespace',
                              default_value=EnvironmentVariable('AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgumentsFromConfigFile(
            name='config_file', source_file=config_file,
            description='Configuration file'),
        DeclareLaunchArgumentsFromConfigFile(
            'camera_calibration_file',
            source_file=camera_calibration_file,
            description='Camera calibration file'),
        Node(
            package='as2_usb_camera_interface',
            executable='as2_usb_camera_interface_node',
            name='usb_camera_interface',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            arguments=['--ros-args', '--log-level',
                       LaunchConfiguration('log_level')],
            emulate_tty=True,
            parameters=[
                LaunchConfigurationFromConfigFile(
                    'config_file',
                    default_file=config_file),
                LaunchConfigurationFromConfigFile(
                    'camera_calibration_file',
                    default_file=camera_calibration_file),
            ],
        ),
    ])
