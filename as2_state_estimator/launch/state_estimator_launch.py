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

"""Launch file for the state estimator node."""

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'
__version__ = '0.1.0'

import os

from ament_index_python.packages import get_package_share_directory
import as2_core.launch_param_utils as as2_utils
from as2_core.launch_plugin_utils import get_available_plugins
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def get_package_config_file():
    """Return the package config file."""
    package_folder = get_package_share_directory('as2_state_estimator')
    return os.path.join(package_folder,
                        'config/state_estimator_default.yaml')


def get_node(context, *args, **kwargs) -> list:
    """
    Get node.

    :param context: Launch context
    :type context: LaunchContext
    :return: List with node
    :rtype: list
    """
    # Get plugin name
    plugin_name = LaunchConfiguration('plugin_name').perform(context)

    # Get plugin configuration file
    package_folder = get_package_share_directory(
        'as2_state_estimator')
    plugin_config_file = os.path.join(package_folder,
                                      'plugins/' +
                                      plugin_name +
                                      '/config/plugin_default.yaml')

    return [
        *as2_utils.declare_launch_arguments(
            'plugin_config_file',
            default_value=plugin_config_file,
            description='Plugin configuration file'),
        Node(
            package='as2_state_estimator',
            executable='as2_state_estimator_node',
            name='state_estimator',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            arguments=['--ros-args', '--log-level',
                       LaunchConfiguration('log_level')],
            emulate_tty=True,
            parameters=[
                *as2_utils.launch_configuration(
                    'config_file',
                    default_value=get_package_config_file()),
                *as2_utils.launch_configuration(
                    'plugin_config_file',
                    default_value=plugin_config_file),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'plugin_name': plugin_name
                },
            ]
        )
    ]


def generate_launch_description() -> LaunchDescription:
    """
    Entry point for launch file.

    :return: Launch description
    :rtype: LaunchDescription
    """
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
        DeclareLaunchArgument('plugin_name',
                              description='Plugin name',
                              choices=get_available_plugins(
                                  'as2_state_estimator')),
        *as2_utils.declare_launch_arguments(
            'config_file',
            default_value=get_package_config_file(),
            description='Configuration file'),
        OpaqueFunction(function=get_node)
    ])
