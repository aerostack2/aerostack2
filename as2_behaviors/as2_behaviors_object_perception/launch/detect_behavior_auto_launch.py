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

"""Launch file for the object perception behavior with auto-launch."""

__authors__ = 'Alba López del Águila'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import os

from ament_index_python.packages import get_package_share_directory
from as2_core.declare_launch_arguments_from_config_file import DeclareLaunchArgumentsFromConfigFile
from as2_core.launch_configuration_from_config_file import LaunchConfigurationFromConfigFile
from as2_core.launch_plugin_utils import get_available_plugins
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
import yaml


PACKAGE = 'as2_behaviors_object_perception'


def _extract_ros_parameters(data: dict) -> dict:
    """Extract ros__parameters from YAML config, handling both /** and node-specific structures."""
    result = {}

    if data is None:
        return result

    if '/**' in data:
        global_params = data['/**']
        if isinstance(global_params, dict):
            if 'ros__parameters' in global_params:
                result.update(global_params['ros__parameters'])
            for key, value in global_params.items():
                if key != 'ros__parameters' and isinstance(value, dict):
                    node_ros_params = value.get('ros__parameters', {})
                    if node_ros_params:
                        result.update(node_ros_params)

    for key, value in data.items():
        if key != '/**' and isinstance(value, dict):
            node_ros_params = value.get('ros__parameters', {})
            if node_ros_params:
                result.update(node_ros_params)

    return result if result else {}


def get_default_config_file() -> str:
    """Return the default configuration file path."""
    return os.path.join(
        get_package_share_directory(PACKAGE),
        'config',
        'config.yaml'
    )


def extract_pipeline_plugins(ros_params: dict) -> list:
    """Extract pipeline plugin names from ROS parameters."""
    pipeline = ros_params.get('pipeline', {})
    stages = pipeline.get('stages', [])
    available_plugins = get_available_plugins(PACKAGE)

    plugins = []
    for stage in stages:
        plugin = pipeline.get(stage, {}).get('plugin')
        if not plugin:
            raise RuntimeError(f"Pipeline stage '{stage}' does not declare a plugin.")
        if plugin not in available_plugins:
            raise RuntimeError(
                f"Pipeline stage '{stage}' requests unknown plugin '{plugin}'. "
                f'Available: {available_plugins}')
        if plugin not in plugins:
            plugins.append(plugin)

    if not plugins:
        raise RuntimeError('No pipeline stages declared in configuration.')
    return plugins


def get_node(context, *args, **kwargs) -> list:
    """Build the behavior node with plugin configurations."""
    package_folder = get_package_share_directory(PACKAGE)
    default_config = get_default_config_file()

    merged_config_file = LaunchConfigurationFromConfigFile(
        'config_file',
        default_file=default_config
    ).perform(context)

    with open(merged_config_file, 'r', encoding='utf-8') as f:
        config_data = yaml.safe_load(f) or {}

    ros_params = _extract_ros_parameters(config_data)
    plugins = extract_pipeline_plugins(ros_params)

    plugin_config_files = [
        os.path.join(package_folder, 'plugins', plugin, 'config', 'plugin_default_config.yaml')
        for plugin in plugins
    ]
    parameters = [path for path in plugin_config_files if os.path.isfile(path)]

    parameters.extend([
        {'use_sim_time': LaunchConfiguration('use_sim_time')},
        merged_config_file,
    ])

    if ros_params.get('use_embedded_camera', False):
        parameters.extend([
            LaunchConfiguration('camera_interface_file'),
            LaunchConfiguration('calibration_file'),
        ])

    return [Node(
        package=PACKAGE,
        executable=f'{PACKAGE}_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=parameters,
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        emulate_tty=True,
    )]


def generate_launch_description() -> LaunchDescription:
    """Entrypoint."""
    package_folder = get_package_share_directory(PACKAGE)
    default_config = get_default_config_file()

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
                              description='Embedded camera driver configuration',
                              default_value=os.path.join(
                                  package_folder, 'config', 'camera_interface.yaml')),
        DeclareLaunchArgument('calibration_file',
                              description='Camera calibration parameters',
                              default_value=os.path.join(
                                  package_folder, 'config', 'camera_calibration.yaml')),
        DeclareLaunchArgumentsFromConfigFile(
            name='config_file',
            source_file=default_config,
            description='Behavior configuration file'),
        OpaqueFunction(function=get_node),

        # Send the goal once the node is up, so the pipeline runs without an
        # external client.
        ExecuteProcess(
            cmd=['bash', '-c', [
                'sleep 5 && ros2 action send_goal /',
                LaunchConfiguration('namespace'),
                '/ObjectPerceptionBehavior as2_msgs/action/DetectObjects ',
                '"{threshold: ', LaunchConfiguration('threshold'),
                ', target_classes: ', LaunchConfiguration('target_classes'), '}"',
            ]],
            output='screen',
            shell=False
        ),
    ])
