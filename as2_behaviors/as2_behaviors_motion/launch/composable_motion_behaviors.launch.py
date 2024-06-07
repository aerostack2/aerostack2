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

"""Composable_motion_behaviors.launch.py."""

__authors__ = 'Rafael Pérez Seguí, Pedro Arias Pérez'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import os

from ament_index_python.packages import get_package_share_directory
import as2_core.launch_param_utils as as2_utils
from as2_core.launch_plugin_utils import get_available_plugins
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def snake_to_camel(text: str) -> str:
    """Convert snake_case to CamelCase."""
    return ''.join(x.capitalize() or '_' for x in text.split('_'))


def generate_launch_description():
    """Launch the AS2 platform behaviors."""
    behaviors = [
        'follow_path',
        'go_to',
        'land',
        'takeoff']

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

    package_folder = get_package_share_directory('as2_behaviors_motion')
    behavior_components = []
    for behavior in behaviors:
        launch_description.append(
            DeclareLaunchArgument(behavior + '_plugin_name', description='Plugin name',
                                  choices=get_available_plugins('as2_behaviors_motion', behavior)))
        behavior_config_file = os.path.join(package_folder,
                                            behavior +
                                            '_behavior/config/config_default.yaml')
        launch_description.extend(
            as2_utils.declare_launch_arguments(
                behavior + '_config_file',
                default_value=behavior_config_file,
                description='Path to behavior config file'))
        composable_node = ComposableNode(
            package='as2_behaviors_motion',
            plugin=snake_to_camel(behavior) + 'Behavior',
            name=snake_to_camel(behavior) + 'Behavior',
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                *as2_utils.launch_configuration(behavior + '_config_file',
                                                default_value=behavior_config_file),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'plugin_name': LaunchConfiguration(behavior + '_plugin_name'),
                }
            ])
        behavior_components.append(composable_node)
    container = ComposableNodeContainer(
        name='behaviors',
        namespace=LaunchConfiguration('namespace'),
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=behavior_components,
        output='screen',
    )
    launch_description.append(container)
    return LaunchDescription(launch_description)
