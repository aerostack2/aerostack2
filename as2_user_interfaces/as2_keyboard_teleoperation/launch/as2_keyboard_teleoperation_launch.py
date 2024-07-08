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
import yaml


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

    config_file = LaunchConfiguration('config_file').perform(context)

    with open(config_file, 'r') as file:
        config_data = yaml.safe_load(file)
        if 'ros__parameters' in config_data.get('/**', {}):
            param_data = {**config_data['/**']['ros__parameters']['teleop_values'],
                          **config_data['/**']['ros__parameters']['teleop_config'],
                          **config_data['/**']['ros__parameters']['node_config']}
        else:
            param_data = {}

    parameters = []
    for key, value in param_data.items():
        if key == 'use_sim_time':
            use_sim_time = str(value).lower()
            continue
        if key == 'verbose':
            verbose = str(value).lower()
            continue
        if key == 'namespace':
            namespace = process_namespace(value)
            continue
        parameters.append(f'--{key}={value}')

    namespace_launch_config = LaunchConfiguration('namespace').perform(context)
    if namespace_launch_config != 'default':
        namespace = process_namespace(namespace_launch_config)
    verbose_launch_config = LaunchConfiguration('verbose').perform(context)
    if verbose_launch_config != 'default':
        if verbose_launch_config.lower() != 'true' and verbose_launch_config.lower() != 'false':
            raise ValueError('Verbose argument must be true or false')
        verbose = verbose_launch_config
    use_sim_time_launch_config = LaunchConfiguration('use_sim_time').perform(context)
    if use_sim_time_launch_config != 'default':
        if use_sim_time_launch_config.lower() != 'true' and use_sim_time_launch_config.lower() != 'false':
            raise ValueError('Use simulation time argument must be true or false')
        use_sim_time = use_sim_time_launch_config

    process = ExecuteProcess(
        cmd=['python3', keyboard_teleop, f'--namespace={namespace}',
             f'--verbose={verbose}', f'--use_sim_time={use_sim_time}'] + parameters,
        name='as2_keyboard_teleoperation',
        output='screen')
    return [process]


def generate_launch_description():
    """Entrypoint launch description method."""
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'namespace',
            default_value='default',
            description='namespaces list.'),
        DeclareLaunchArgument(
            'config_file',
            default_value=get_config_file(),
            description='Config file path.'),
        DeclareLaunchArgument(
            'verbose',
            default_value='default',
            description='Launch in verbose mode.'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='default',
            description='Use simulation time.'),
        OpaqueFunction(function=launch_teleop),
    ])
