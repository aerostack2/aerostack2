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

"""Launch file for point gimbal behavior node."""

import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable

import yaml


def get_default_launch(filepath: str):
    """Get default launch arguments and configurations from config file"""
    with open(filepath, 'r', encoding='utf-8') as file:
        lines = file.read()
        params = yaml.safe_load(lines)

    dl_args = []
    configs = {}
    for name, value in params['/**']['ros__parameters'].items():
        if isinstance(value, dict):
            continue  # skip dictionaries aka nested parameters
        i = lines.find(name)
        j = lines.find('#', i) + 1
        k = lines.find('\n', i)
        k = len(lines) if k == -1 else k
        descrip = 'no description given' if j > k else lines[j+1:k]
        dl_args.append(DeclareLaunchArgument(
            name, default_value=str(value), description=descrip))
        configs[name] = LaunchConfiguration(name)

    return dl_args, configs


def generate_launch_description():
    """Launch point gimbal behavior node."""
    config = os.path.join(get_package_share_directory('as2_behaviors_perception'),
                          'point_gimbal_behavior/config/config_default.yaml')
    dl_args, configs = get_default_launch(config)

    return LaunchDescription([
        DeclareLaunchArgument('namespace', description='Drone namespace',
                              default_value=EnvironmentVariable('AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info',
                              description='Log Severity Level'),
        DeclareLaunchArgument(
            'config_file', description='Config file', default_value=config),
        *dl_args,
        Node(
            package='as2_behaviors_perception',
            executable='point_gimbal_behavior_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            arguments=['--ros-args', '--log-level',
                       LaunchConfiguration('log_level')],
            parameters=[
                {LaunchConfiguration('config_file')},
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    **configs
                },
            ],
            emulate_tty=True,
        ),
    ])
