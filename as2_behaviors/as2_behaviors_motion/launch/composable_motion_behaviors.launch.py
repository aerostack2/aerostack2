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

"""
composable_motion_behaviors.launch.py
"""

import os
from typing import List
from xml.etree import ElementTree
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def get_available_plugins(package_name: str, plugin_type: str) -> List[str]:
    """
    Parse plugins.xml file from package and return a list of plugins from a specific type
    """
    plugins_file = os.path.join(
        get_package_share_directory(package_name),
        'plugins.xml'
    )
    root = ElementTree.parse(plugins_file).getroot()

    available_plugins = []
    for class_element in root.findall('class'):
        if plugin_type in class_element.attrib['type']:
            available_plugins.append(
                class_element.attrib['type'].split('::')[0])
    return available_plugins


def snake_to_camel(text: str) -> str:
    """Converts snake_case to CamelCase"""
    return ''.join(x.capitalize() or '_' for x in text.split('_'))


def generate_launch_description() -> LaunchDescription:
    """Generate launch description with multiple behaviors components."""
    behaviors_list = [
        "follow_path",
        "go_to",
        "land",
        "takeoff"
    ]

    declare_namespace = DeclareLaunchArgument(
        'namespace', description='Drone namespace'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false'
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace)
    ld.add_action(declare_use_sim_time)
    behavior_components = []
    for behavior in behaviors_list:
        ld.add_action(
            DeclareLaunchArgument(behavior + '_plugin_name', description='Plugin name',
                                  choices=get_available_plugins('as2_behaviors_motion', behavior))
        )
        default_config_file = os.path.join(
            get_package_share_directory('as2_behaviors_motion'),
            'config/' + behavior + '_behavior/config_default.yaml'
        )
        ld.add_action(
            DeclareLaunchArgument(behavior + '_config_file', default_value=default_config_file,
                                  description='Path to behavior config file')
        )
        behavior = ComposableNode(
            namespace=LaunchConfiguration('namespace'),
            package='as2_behaviors_motion',
            plugin=snake_to_camel(behavior) + 'Behavior',
            name=snake_to_camel(behavior) + 'Behavior',
            parameters=[
                {"use_sim_time": LaunchConfiguration('use_sim_time')},
                {'plugin_name':  LaunchConfiguration(
                    behavior + '_plugin_name')},
                LaunchConfiguration(behavior + '_config_file')
            ])
        behavior_components.append(behavior)

    container = ComposableNodeContainer(
        name='behaviors',
        namespace=LaunchConfiguration('namespace'),
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=behavior_components,
        output='screen',
    )
    ld.add_action(container)
    return ld
