"""Keyboard Teleopration launch."""

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

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction


def process_namespace(namespace: str):
    """Process namespace."""
    if ',' in namespace:
        ns_list = [ns.replace(" ", "") for ns in namespace.split(',')]
    elif ':' in namespace:
        ns_list = [ns.replace(" ", "") for ns in namespace.split(':')]
    else:
        ns_list = [ns.replace(" ", "") for ns in namespace.split(' ')]
    return ','.join(ns_list)


def get_config_file():
    """Get config file path."""
    return os.path.join(get_package_share_directory('as2_keyboard_teleoperation'),
                         'config', 'teleop_values_config.yaml')


def launch_teleop(context):
    """Teleop python process."""
    package_folder = get_package_share_directory(
        'as2_keyboard_teleoperation')

    keyboard_teleop = os.path.join(package_folder, 'keyboard_teleoperation.py')

    namespace = LaunchConfiguration('namespace').perform(context)
    namespace = process_namespace(namespace)
    verbose = LaunchConfiguration('verbose').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    config_file = LaunchConfiguration('config_file').perform(context)

    process = ExecuteProcess(
        cmd=['python3', keyboard_teleop, namespace, verbose, use_sim_time, config_file],
        name='as2_keyboard_teleoperation',
        output='screen')
    return [process]


def generate_launch_description():
    """Entrypoint launch description method."""
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'namespace',
            description='namespaces list.'),
        DeclareLaunchArgument(
            'config_file',
            default_value=get_config_file(),
            description='Config file path.'),
        DeclareLaunchArgument(
            'verbose',
            default_value='false',
            choices=['true', 'false'],
            description='Launch in verbose mode.'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Use simulation time.'),
        DeclareLaunchArgument(
            'keyboard_teleoperation_config_file',
            default_value='config_values.py',
            description='Keyboard teleoperation configuration file.'),
        OpaqueFunction(function=launch_teleop),
    ])
