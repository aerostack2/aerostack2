#!/usr/bin/env python3

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

"""Launch file for the state estimator node."""

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'
__version__ = '0.1.0'

import logging
import sys

from ament_index_python.packages import PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

FORMAT = '[%(levelname)s] [launch]: %(message)s'
logging.basicConfig(format=FORMAT)


def get_state_estimator_node(context):
    """Return the state estimator node."""
    plugin_name = LaunchConfiguration('plugin_name').perform(context)
    if not plugin_name:
        logging.critical('Plugin not set.')
        sys.exit(1)

    plugin_config_file = LaunchConfiguration(
        'plugin_config_file').perform(context)

    parameters = [{
        'plugin_name': plugin_name,
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'base_frame': LaunchConfiguration('base_frame'),
        'global_ref_frame': LaunchConfiguration('global_ref_frame'),
        'odom_frame': LaunchConfiguration('odom_frame'),
        'map_frame': LaunchConfiguration('map_frame'),
        'rigid_body_name': LaunchConfiguration('rigid_body_name'),
        'mocap_topic': LaunchConfiguration('mocap_topic'),
        'twist_smooth_filter_cte': LaunchConfiguration('twist_smooth_filter_cte'),
        'orientation_smooth_filter_cte': LaunchConfiguration('orientation_smooth_filter_cte'),
    }]

    if not plugin_config_file:
        try:
            plugin_config_file = PathJoinSubstitution([
                FindPackageShare(plugin_name),
                'config', 'default_state_estimator.yaml'
            ])
            plugin_config_file.perform(context)
        except PackageNotFoundError:
            plugin_config_file = PathJoinSubstitution([
                FindPackageShare('as2_state_estimator'),
                'plugins/' + plugin_name + '/config', 'default_state_estimator.yaml'
            ])

    parameters.append(plugin_config_file)

    node = Node(
        package='as2_state_estimator',
        executable='as2_state_estimator_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=parameters,
        output='screen',
        emulate_tty=True
    )

    return [node]


def generate_launch_description():
    """Return the launch description."""
    launch_description = LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=EnvironmentVariable(
            'AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgument('plugin_name'),
        DeclareLaunchArgument('plugin_config_file', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('global_ref_frame', default_value='earth'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('rigid_body_name', default_value=''),
        DeclareLaunchArgument('mocap_topic', default_value=''),
        DeclareLaunchArgument(
            'twist_smooth_filter_cte', default_value='0.1',
            description='Smoothing constant for the twist filter. ' +
            'The closer to 0, the smoother the output, while the closer to 1, ' +
            'the more responsive the output (1 is equivalent to no smoothing). ' +
            'Only used in the mocap plugin.'),
        DeclareLaunchArgument(
            'orientation_smooth_filter_cte', default_value='1.0',
            description='Smoothing constant for the orientation filter. ' +
            'The closer to 0, the smoother the output, while the closer to 1, ' +
            'the more responsive the output (1 is equivalent to no smoothing). ' +
            'Only used in the mocap plugin.'),
        OpaqueFunction(function=get_state_estimator_node)
    ])

    return launch_description
