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

"""Launch file for the motion behavior."""

__authors__ = 'Rafael Pérez Seguí, Pedro Arias Pérez'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import os

from ament_index_python.packages import get_package_share_directory
from as2_core.declare_launch_arguments_from_config_file import DeclareLaunchArgumentsFromConfigFile
from as2_core.launch_configuration_from_config_file import LaunchConfigurationFromConfigFile
from as2_core.launch_plugin_utils import get_available_plugins
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
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


def override_plugin_name_in_context(context: LaunchContext, behavior_name: str):
    """Override plugin_name in the context from config_file if it is not provided as argument."""
    plugin_name = LaunchConfiguration(behavior_name + '_plugin_name').perform(context)
    if plugin_name == '':
        config_file = LaunchConfiguration('config_file').perform(context)
        with open(config_file, 'r') as file:
            config = yaml.safe_load(file)
        plugin_names = recursive_search(config, behavior_name + '_plugin_name')
        for plugin_name in plugin_names:
            if plugin_name in get_available_plugins('as2_behaviors_motion'):
                break

    if plugin_name == '':
        raise RuntimeError('No plugin_name provided or not found in config_file.')

    context.launch_configurations[behavior_name + '_plugin_name'] = plugin_name
    return


def override_behavior_default_config_file(context: LaunchContext, behavior_name: str):
    """Override behavior config file in context from the common one."""
    if LaunchConfiguration('config_file').perform(context) == '':
        return
    context.launch_configurations[
        behavior_name + '_config_file'] = LaunchConfiguration('config_file').perform(context)
    # Use sim time back to string
    context.launch_configurations['use_sim_time'] = str(LaunchConfiguration(
        'use_sim_time').perform(context)).lower()
    return


def get_behavior_launch_description(behavior_name: str) -> list:
    plugin_choices = get_available_plugins('as2_behaviors_motion', behavior_name)
    plugin_choices.append('')
    package_folder = get_package_share_directory('as2_behaviors_motion')
    behavior_config_file = os.path.join(package_folder,
                                        behavior_name +
                                        '_behavior/config/config_default.yaml')
    launch_description = [
        DeclareLaunchArgument(
            behavior_name + '_plugin_name',
            default_value='',
            description='Plugin name. If empty, it must be declared in config file.',
            choices=plugin_choices),
        OpaqueFunction(function=override_plugin_name_in_context, args=[behavior_name]),
        DeclareLaunchArgumentsFromConfigFile(
            name=behavior_name + '_config_file', source_file=behavior_config_file,
            description='Path to behavior configuration file'),
        OpaqueFunction(function=override_behavior_default_config_file, args=[behavior_name]),
        Node(
            package='as2_behaviors_motion',
            executable=behavior_name + '_behavior_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            emulate_tty=True,
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'plugin_name': LaunchConfiguration(behavior_name + '_plugin_name'),
                },
                LaunchConfigurationFromConfigFile(
                    behavior_name + '_config_file', behavior_config_file),
            ])
    ]
    return launch_description


def generate_launch_description():
    """Launch basic motion behaviors."""
    behaviors = [
        'follow_path',
        'go_to',
        'land',
        'takeoff'
    ]

    launch_description = []
    launch_description.append(
        DeclareLaunchArgument('log_level',
                              description='Logging level',
                              default_value='info'))
    launch_description.append(
        DeclareLaunchArgument('use_sim_time',
                              description='Use simulation clock if true',
                              default_value='false'))
    launch_description.append(
        DeclareLaunchArgument('namespace',
                              description='Drone namespace',
                              default_value=EnvironmentVariable(
                                  'AEROSTACK2_SIMULATION_DRONE_ID')))
    launch_description.append(
        DeclareLaunchArgument('config_file',
                              description='Path to behaviors configuration file',
                              default_value=''))

    for behavior in behaviors:
        launch_description += get_behavior_launch_description(behavior)

    return LaunchDescription(launch_description)
