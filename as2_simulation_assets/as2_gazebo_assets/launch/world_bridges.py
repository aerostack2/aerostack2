"""world_bridges.py."""

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


__authors__ = 'Pedro Arias Pérez, Javier Melero Deza, Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'
__version__ = '0.1.0'

from as2_gazebo_assets.bridges import bridges as gz_bridges

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import json

from as2_gazebo_assets.bridges import bridges as gz_bridges
from as2_gazebo_assets.bridges import custom_bridges as gz_custom_bridges
from launch_ros.actions import Node


def world_bridges(context):
    """
    Return world bridges.

    Mainly clock if sim_time enabled.
    """
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    config_file = LaunchConfiguration(
        'simulation_config_file').perform(context)
    with open(config_file, 'r', encoding='utf-8') as stream:
        config = json.load(stream)
    bridges = [
    ]
    bridges.append(gz_bridges.world_control(config['world_name']))
#    if use_sim_time:
#        bridges.append(gz_bridges.clock())
    nodes = []
    node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace='world',
        output='screen',
        arguments=[bridge.argument() for bridge in bridges],
        remappings=[bridge.remapping() for bridge in bridges]
    )
    nodes.append(node)
    nodes.append(gz_custom_bridges.set_pose_bridge(config['world_name']))

    return nodes


def generate_launch_description():
    """Generate Launch description with world bridges."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            description='Make objects publish tfs in sys clock time or sim time'
        ),
        OpaqueFunction(function=world_bridges)
    ])
