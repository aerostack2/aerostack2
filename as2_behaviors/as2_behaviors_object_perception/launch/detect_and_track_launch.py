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

"""Launch the detector_yolo_gates -> tracker_tf perception pipeline."""

__authors__ = 'Alba López del Águila'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from as2_core.declare_launch_arguments_from_config_file import DeclareLaunchArgumentsFromConfigFile
from as2_core.launch_configuration_from_config_file import LaunchConfigurationFromConfigFile
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Entrypoint."""
    package_folder = get_package_share_directory(
        'as2_behaviors_object_perception')
    pipeline_config_file = os.path.join(
        package_folder, 'config/config_detector_tracker_pipeline.yaml')

    calibration_file = os.path.join(
        package_folder, 'config/arducam_calibration.yaml')

    common_arguments = ['--ros-args', '--log-level',
                        LaunchConfiguration('log_level')]
    common_parameters = [
        {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }
    ]

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
        DeclareLaunchArgument('pipeline_config_file',
                              description='Behavior configuration file',
                              default_value=pipeline_config_file),
        DeclareLaunchArgumentsFromConfigFile(
            name='calibration_file', source_file=calibration_file,
            description='Arducam calibration file'),
        Node(
            package='as2_behaviors_object_perception',
            executable='as2_behaviors_object_perception_node',
            name='object_perception',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            arguments=common_arguments,
            emulate_tty=True,
            parameters=common_parameters + [

                LaunchConfiguration('pipeline_config_file'),

                LaunchConfigurationFromConfigFile(
                    'calibration_file', default_file=calibration_file),
            ]
        )
    ])
