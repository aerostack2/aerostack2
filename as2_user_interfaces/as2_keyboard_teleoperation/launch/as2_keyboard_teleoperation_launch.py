"""Keyboard Teleopration launch."""

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

import os

from ament_index_python.packages import get_package_share_directory
from as2_core.declare_launch_arguments_from_config_file import DeclareLaunchArgumentsFromConfigFile
from as2_core.launch_configuration_from_config_file import LaunchConfigurationFromConfigFile
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import yaml


def process_list(namespace: str):
    """Process namespace."""
    if ',' in namespace:
        ns_list = [ns.replace(' ', '') for ns in namespace.split(',')]
    elif ':' in namespace:
        ns_list = [ns.replace(' ', '') for ns in namespace.split(':')]
    else:
        ns_list = [ns.replace(' ', '') for ns in namespace.split(' ')]
    return ','.join(ns_list)


def get_config_file():
    """Get config file path."""
    return os.path.join(get_package_share_directory('as2_keyboard_teleoperation'),
                        'config', 'teleop_values_config.yaml')


def launch_teleop(context: LaunchContext):
    """Teleop python process."""
    package_folder = get_package_share_directory(
        'as2_keyboard_teleoperation')

    keyboard_teleop = os.path.join(package_folder, 'keyboard_teleoperation.py')

    namespace = LaunchConfiguration('namespace').perform(context)
    if namespace != '':
        namespace = process_list(namespace)
    verbose = LaunchConfiguration('verbose').perform(context).lower()
    if verbose != '':
        if verbose != 'true' and verbose != 'false':
            raise ValueError('Verbose argument must be true or false')
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower()
    if use_sim_time != '':
        if use_sim_time != 'true' and use_sim_time != 'false':
            raise ValueError('Use simulation time argument must be true or false')

    tmp_file = LaunchConfigurationFromConfigFile(
        'config_file', get_config_file()).perform(context)

    with open(tmp_file, 'r') as file:
        config_data = yaml.safe_load(file)
        if 'ros__parameters' in config_data.get('/**', {}):
            param_data = config_data['/**']['ros__parameters']
        else:
            param_data = {}

    context.launch_configurations.update(param_data)

    if 'config_file' in context.launch_configurations:
        context.launch_configurations.pop('config_file')
    if '/**' in context.launch_configurations:
        context.launch_configurations.pop('/**')

    if use_sim_time != '':
        context.launch_configurations['use_sim_time'] = use_sim_time
    if verbose != '':
        context.launch_configurations['verbose'] = verbose
    if namespace != '':
        context.launch_configurations['namespace'] = namespace

    if 'modules' in context.launch_configurations:
        context.launch_configurations['modules'] = process_list(
            context.launch_configurations['modules'])

    if 'namespace' in context.launch_configurations:
        context.launch_configurations['namespace'] = process_list(
            context.launch_configurations['namespace'])

    # Real defaults from node config file (no other way to do it)
    if context.launch_configurations['namespace'] == '':
        context.launch_configurations['namespace'] = 'drone0'

    if context.launch_configurations['use_sim_time'] == '':
        context.launch_configurations['use_sim_time'] = 'true'

    if context.launch_configurations['verbose'] == '':
        context.launch_configurations['verbose'] = 'false'

    parameters = []
    for key, value in context.launch_configurations.items():
        parameters.append(f'--{key}={str(value).lower()}') if key \
            in ['use_sim_time', 'verbose'] else parameters.append(f'--{key}={value}')

    process = ExecuteProcess(
        cmd=['python3', keyboard_teleop] + parameters,
        name='as2_keyboard_teleoperation',
        output='screen')
    return [process]


def generate_launch_description():
    """Entrypoint launch description method."""
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='namespaces list.'),
        DeclareLaunchArgument(
            'verbose',
            default_value='',
            description='Launch in verbose mode.'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='',
            description='Use simulation time.'),
        DeclareLaunchArgumentsFromConfigFile(
            'config_file',
            source_file=get_config_file(),
            description='Config file path.'),
        OpaqueFunction(function=launch_teleop),
    ])
