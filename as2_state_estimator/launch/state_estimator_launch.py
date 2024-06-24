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

"""Launch file for the state estimator node."""

__authors__ = 'Pedro Arias Pérez, Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import os

from ament_index_python.packages import get_package_share_directory
from as2_core.declare_launch_arguments_from_config_file import DeclareLaunchArgumentsFromConfigFile
from as2_core.launch_configuration_from_config_file import LaunchConfigurationFromConfigFile
from as2_core.launch_plugin_utils import get_available_plugins
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import yaml


def recursive_search(data_dict, target_key, result=None):
    """Search for a target key in a nested dictionary or list."""
    if result is None:
        result = []

    if isinstance(data_dict, dict):
        for key, value in data_dict.items():
            if key == target_key:
                result.append(value)
            else:
                recursive_search(value, target_key, result)
    elif isinstance(data_dict, list):
        for item in data_dict:
            recursive_search(item, target_key, result)

    return result


def override_plugin_name_in_context(context):
    """Override plugin_name in the context from config_file if it is not provided as argument."""
    plugin_name = LaunchConfiguration('plugin_name').perform(context)
    if plugin_name == '':
        config_file = LaunchConfiguration('config_file').perform(context)
        with open(config_file, 'r') as file:
            config = yaml.safe_load(file)
        plugin_names = recursive_search(config, 'plugin_name')
        for plugin_name in plugin_names:
            if plugin_name in get_available_plugins('as2_state_estimator'):
                break

    if plugin_name == '':
        raise RuntimeError('No plugin_name provided or not found in config_file.')

    context.launch_configurations['plugin_name'] = plugin_name
    return


def get_launch_description_from_plugin(
        plugin_name: str | LaunchConfiguration) -> LaunchDescription:
    """Get LaunchDescription from plugin."""
    package_folder = get_package_share_directory('as2_state_estimator')
    config_file = os.path.join(package_folder,
                               'config/state_estimator_default.yaml')
    if isinstance(plugin_name, LaunchConfiguration):
        plugin_config_file = PathJoinSubstitution([
            package_folder,
            'plugins', LaunchConfiguration('plugin_name'), 'config/plugin_default.yaml'
        ])
    elif plugin_name == '':
        plugin_config_file = ''
    else:
        plugin_config_file = os.path.join(package_folder,
                                          'plugins/' + plugin_name + '/config/plugin_default.yaml')
    return [
        DeclareLaunchArgument('log_level',
                              description='Logging level',
                              default_value='info'),
        DeclareLaunchArgument('use_sim_time',
                              description='Use simulation clock if true',
                              default_value='false'),
        DeclareLaunchArgument('namespace',
                              description='Drone namespace',
                              default_value=EnvironmentVariable('AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgumentsFromConfigFile(
            name='config_file', source_file=config_file,
            description='Configuration file'),
        DeclareLaunchArgumentsFromConfigFile(
            name='plugin_config_file', source_file=plugin_config_file,
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
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'plugin_name': plugin_name
                },
                LaunchConfigurationFromConfigFile(
                    'config_file',
                    default_file=config_file),
                LaunchConfigurationFromConfigFile(
                    'plugin_config_file',
                    default_file=plugin_config_file),
            ]
        )
    ]


def generate_launch_description() -> LaunchDescription:
    """Entry point for launch file."""
    plugin_choices = get_available_plugins('as2_state_estimator')
    plugin_choices.append('')
    ld = [
        DeclareLaunchArgument(
            'plugin_name',
            default_value='',
            description='Plugin name. If empty, it must be declared in config file.',
            choices=plugin_choices),
    ]
    ld.append(OpaqueFunction(function=override_plugin_name_in_context))
    ld.extend(get_launch_description_from_plugin(LaunchConfiguration('plugin_name')))
    return LaunchDescription(ld)
