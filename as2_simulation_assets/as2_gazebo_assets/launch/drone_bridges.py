"""
drone_bridges.py
"""

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

import json
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo, Shutdown
from launch.substitutions import LaunchConfiguration

from as2_gazebo_assets.world import World


def drone_bridges(context):
    """Return drone bridges"""
    namespace = LaunchConfiguration('namespace').perform(context)
    config_file = LaunchConfiguration(
        'simulation_config_file').perform(context)

    with open(config_file, 'r', encoding='utf-8') as stream:
        config = json.load(stream)
        world = World(**config)

    nodes = []
    for drone_model in world.drones:
        if drone_model.model_name == namespace:  # only the one desired
            bridges, custom_bridges = drone_model.bridges(world.world_name)
            nodes.append(Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                namespace=drone_model.model_name,
                output='screen',
                arguments=[bridge.argument() for bridge in bridges],
                remappings=[bridge.remapping() for bridge in bridges]
            ))
            nodes += custom_bridges

    if not nodes:
        return [
            LogInfo(msg="Gazebo Ignition bridge creation failed."),
            LogInfo(msg=f"Drone ID: {namespace} not found in {config_file}."),
            Shutdown(reason="Aborting..")]
    return nodes


def generate_launch_description():
    """Generate Launch description with world bridges
    """
    return LaunchDescription([
        DeclareLaunchArgument(
            'simulation_config_file',
            description='YAML configuration file to spawn'
        ),
        DeclareLaunchArgument(
            'namespace',
            description='Drone ID to create bridges'
        ),
        OpaqueFunction(function=drone_bridges)
    ])
