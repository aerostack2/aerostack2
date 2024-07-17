"""swarm_viz.launch.py."""


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
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
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


__authors__ = 'Pedro Arias Pérez'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def get_viz_nodes(context):
    ns_list = LaunchConfiguration('namespace_list').perform(context).split(',')
    ld = []
    for ns in ns_list:
        if not ld:
            # First launcher includes rviz
            drone = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory('as2_visualization'), 'launch'
                        ),
                        '/as2_visualization.launch.py',
                    ]
                ),
                launch_arguments={
                    'rviz_config': LaunchConfiguration('rviz_config'),
                    'namespace': ns,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'record_length': LaunchConfiguration('record_length'),
                }.items(),
            )
            ld.append(drone)
        else:
            drone = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory('as2_visualization'), 'launch'
                        ),
                        '/as2_visualization.launch.py',
                    ]
                ),
                launch_arguments={
                    'namespace': ns,
                    'rviz': 'false',
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'record_length': LaunchConfiguration('record_length'),
                }.items(),
            )
            ld.append(drone)
    return ld


def generate_launch_description():
    default_rviz_config = os.path.join(
        get_package_share_directory('as2_visualization'), 'config', 'as2_default.rviz'
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'namespace_list',
                description='List of drone namespaces separated by commas. E.g: "drone0, drone1".',
            ),
            DeclareLaunchArgument(
                'use_sim_time', default_value='false', description='Use simulation time'
            ),
            DeclareLaunchArgument(
                'rviz_config',
                default_value=default_rviz_config,
                description='RViz configuration file.',
            ),
            DeclareLaunchArgument(
                'record_length',
                default_value='500',
                description='Length for last poses.',
            ),
            OpaqueFunction(function=get_viz_nodes),
        ]
    )
