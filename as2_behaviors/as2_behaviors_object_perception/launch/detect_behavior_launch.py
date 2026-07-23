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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def get_node(context, *args, **kwargs) -> list:
    """Build the behavior node, adding the camera driver files only in embedded mode."""
    package_folder = get_package_share_directory(
        'as2_behaviors_object_perception')

    # Falls back to the plugin's own default config, installed by CMake under
    # share/<pkg>/plugins/<plugin_name>/config/.
    plugin_config_file = LaunchConfiguration('plugin_config_file').perform(context)
    if not plugin_config_file:
        plugin_name = LaunchConfiguration('plugin_name').perform(context)
        plugin_config_file = os.path.join(
            package_folder, 'plugins', plugin_name, 'config',
            'plugin_default_config.yaml')

    parameters = [
        {'use_sim_time': LaunchConfiguration('use_sim_time')},
        {'use_embedded_camera': LaunchConfiguration('use_embedded_camera')},
        LaunchConfiguration('config_file'),
        plugin_config_file,
    ]

    # In topic mode the camera driver is external: its params and its calibration
    # reach the behavior through camera_info, so these files are not needed.
    if IfCondition(LaunchConfiguration('use_embedded_camera')).evaluate(context):
        parameters += [
            LaunchConfiguration('camera_interface_file'),
            LaunchConfiguration('calibration_file'),
        ]

    return [Node(
        package='as2_behaviors_object_perception',
        executable='as2_behaviors_object_perception_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=parameters,
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        emulate_tty=True,
    )]


def generate_launch_description() -> LaunchDescription:
    """Entrypoint."""
    package_folder = get_package_share_directory(
        'as2_behaviors_object_perception')

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
        DeclareLaunchArgument('config_file',
                              description='Behavior configuration file',
                              default_value=os.path.join(package_folder,
                                                         'config/config.yaml')),
        DeclareLaunchArgument('plugin_name',
                              description='Detection plugin to load',
                              default_value='aruco'),
        DeclareLaunchArgument('plugin_config_file',
                              description='Detection plugin configuration file. '
                                          'If empty, the default one is used',
                              default_value=''),
        DeclareLaunchArgument('use_embedded_camera',
                              description='Open the camera device from the behavior itself. '
                                          'If false, images come from a topic',
                              default_value='false'),
        DeclareLaunchArgument('camera_interface_file',
                              description='Embedded camera driver params',
                              default_value=os.path.join(
                                  package_folder, 'config/camera_interface.yaml')),
        DeclareLaunchArgument('calibration_file',
                              description='Camera calibration (camera_info) file',
                              default_value=os.path.join(
                                  package_folder, 'config/camera_calibration.yaml')),
        OpaqueFunction(function=get_node),
    ])
