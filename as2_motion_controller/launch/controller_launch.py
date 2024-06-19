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

"""Launch file for the controller manager node."""

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
    package_folder = get_package_share_directory('as2_motion_controller')
    return os.path.join(package_folder,
                        'config/motion_controller_default.yaml')


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
        'as2_motion_controller')
    plugin_config_file = os.path.join(package_folder,
                                      'plugins/' +
                                      plugin_name +
                                      '/config/controller_default.yaml')

    plugin_available_modes_config_file = os.path.join(package_folder,
                                                      'plugins/' +
                                                      plugin_name +
                                                      '/config/available_modes.yaml')

    return [
        DeclareLaunchArgument(
            'plugin_available_modes_config_file',
            description='Plugin available modes configuration file',
            default_value=plugin_available_modes_config_file),
        *as2_utils.declare_launch_arguments(
            'plugin_config_file',
            default_value=plugin_config_file,
            description='Plugin configuration file'),
        Node(
            package='as2_motion_controller',
            executable='as2_motion_controller_node',
            name='controller_manager',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            arguments=['--ros-args', '--log-level',
                       LaunchConfiguration('log_level')],
            emulate_tty=True,
            parameters=[
                *as2_utils.launch_configuration(
                    'motion_controller_config_file',
                    default_value=get_package_config_file()),
                *as2_utils.launch_configuration(
                    'plugin_config_file',
                    default_value=plugin_config_file),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'plugin_name': plugin_name,
                    'plugin_available_modes_config_file': LaunchConfiguration(
                        'plugin_available_modes_config_file')
                },
            ]
        )
    ]


def generate_launch_description():
    """Return the launch description."""
    launch_description = LaunchDescription([
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
                                  'as2_motion_controller')),
        *as2_utils.declare_launch_arguments(
            'motion_controller_config_file',
            default_value=get_package_config_file(),
            description='Configuration file'),
        OpaqueFunction(function=get_node)
    ])

    return launch_description
