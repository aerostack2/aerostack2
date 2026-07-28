# Copyright 2025 Universidad Politécnica de Madrid
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

"""Launch file for the detect behavior."""

__authors__ = 'Guillermo GP-Lenza'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import os

from ament_index_python.packages import get_package_share_directory
from as2_core.declare_launch_arguments_from_config_file import DeclareLaunchArgumentsFromConfigFile
from as2_core.launch_configuration_from_config_file import LaunchConfigurationFromConfigFile
from as2_core.launch_plugin_utils import get_available_plugins
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
import yaml


PACKAGE = 'as2_behaviors_object_perception'


def get_config_file() -> str:
    """Return the default behavior configuration file."""
    return os.path.join(get_package_share_directory(PACKAGE), 'config', 'config.yaml')


def get_pipeline_plugins(config_file: str) -> list:
    """Return the plugins named by the pipeline stages, in stage order and without repeats."""
    with open(config_file, 'r', encoding='utf-8') as file:
        params = yaml.safe_load(file)['/**']['ros__parameters']

    pipeline = params.get('pipeline', {})
    available_plugins = get_available_plugins(PACKAGE)
    plugins = []
    for stage in pipeline.get('stages', []):
        plugin = pipeline.get(stage, {}).get('plugin')
        if plugin is None:
            raise RuntimeError(f"Pipeline stage '{stage}' does not declare a plugin.")
        if plugin not in available_plugins:
            raise RuntimeError(
                f"Pipeline stage '{stage}' requests unknown plugin '{plugin}'. "
                f'Available plugins: {available_plugins}')
        if plugin not in plugins:
            plugins.append(plugin)

    if not plugins:
        raise RuntimeError(f'No pipeline stages declared in {config_file}.')
    return plugins


def get_node(context, *args, **kwargs) -> list:
    """Build the behavior node, resolving the plugin configs from the pipeline stages."""
    package_folder = get_package_share_directory(PACKAGE)
    config_file = get_config_file()

    # Merges the default config with the user one and with the launch arguments,
    # and writes the result to a temporary file.
    merged_config_file = LaunchConfigurationFromConfigFile(
        'config_file', default_file=config_file).perform(context)

    # Each stage names its plugin, so the plugin default configs come from the
    # pipeline itself. Plugin parameters live under a per-plugin namespace, so
    # several of them can coexist, and the behavior config (loaded afterwards)
    # can override any of them.
    plugin_config_files = [
        os.path.join(package_folder, 'plugins', plugin, 'config', 'plugin_default_config.yaml')
        for plugin in get_pipeline_plugins(merged_config_file)
    ]
    parameters = [path for path in plugin_config_files if os.path.isfile(path)]
    parameters += [
        {'use_sim_time': LaunchConfiguration('use_sim_time')},
        merged_config_file,
    ]

    # In topic mode the camera driver is external: its params and its calibration
    # reach the behavior through camera_info, so these files are not needed.
    with open(merged_config_file, 'r', encoding='utf-8') as file:
        use_embedded_camera = yaml.safe_load(
            file)['/**']['ros__parameters'].get('use_embedded_camera', False)
    if use_embedded_camera:
        parameters += [
            LaunchConfiguration('camera_interface_file'),
            LaunchConfiguration('calibration_file'),
        ]

    return [Node(
        package=PACKAGE,
        executable=f'{PACKAGE}_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=parameters,
        output='screen',
        arguments=['--ros-args', '--log-level',
                   LaunchConfiguration('log_level')],
        emulate_tty=True,
    )]


def generate_launch_description() -> LaunchDescription:
    """Entrypoint."""
    package_folder = get_package_share_directory(PACKAGE)

    return LaunchDescription([
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
        DeclareLaunchArgument('camera_interface_file',
                              description='Embedded camera driver params',
                              default_value=os.path.join(
                                  package_folder, 'config', 'camera_interface.yaml')),
        DeclareLaunchArgument('calibration_file',
                              description='Camera calibration (camera_info) file',
                              default_value=os.path.join(
                                  package_folder, 'config', 'camera_calibration.yaml')),
        DeclareLaunchArgumentsFromConfigFile(
            name='config_file', source_file=get_config_file(),
            description='Behavior configuration file'),
        OpaqueFunction(function=get_node),
    ])
