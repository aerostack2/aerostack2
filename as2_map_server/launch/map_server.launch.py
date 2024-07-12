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

"""as2_map_server launch file."""

from __future__ import annotations

import os
from xml.etree import ElementTree

from ament_index_python.packages import get_package_share_directory
from as2_core.launch_param_utils import declare_launch_arguments, launch_configuration
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def get_available_plugins(package_name: str, plugin_type: str = '') -> list[str]:
    """Parse plugins.xml file from package and return a list of plugins from a specific type."""
    plugins_file = os.path.join(
        get_package_share_directory(package_name),
        'plugins.xml'
    )
    root = ElementTree.parse(plugins_file).getroot()
    root = root.find('library') if root.tag == 'class_libraries' else root

    available_plugins = []
    for class_element in root.findall('class'):
        if plugin_type in class_element.attrib['type']:
            available_plugins.append(
                class_element.attrib['type'].split('::')[0])
    return available_plugins


def generate_launch_description():
    """Launcher entrypoint."""
    config_default = os.path.join(get_package_share_directory('as2_map_server'),
                                  'config/plugin_default.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('namespace',
                              default_value=EnvironmentVariable(
                                  'AEROSTACK2_SIMULATION_DRONE_ID'),
                              description='Drone namespace'),
        DeclareLaunchArgument('plugin_name', description='Plugin name',
                              choices=get_available_plugins('as2_map_server')),
        *declare_launch_arguments('config_file', config_default, 'Path to the configuration file'),
        Node(
            package='as2_map_server',
            executable='as2_map_server_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            emulate_tty=True,
            parameters=[
                *launch_configuration('config_file', config_default),
                {'use_sim_time': LaunchConfiguration('use_sim_time'),
                 'plugin_name': LaunchConfiguration('plugin_name')},
            ]
        )
    ])
