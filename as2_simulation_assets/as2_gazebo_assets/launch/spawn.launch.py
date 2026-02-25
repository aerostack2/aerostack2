"""spawn.launch.py file."""

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


__authors__ = 'Pedro Arias Pérez'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import json

from typing import List

from as2_gazebo_assets.utils.launch_exception import InvalidSimulationConfigFile
from as2_gazebo_assets.world import spawn_args, World

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def spawn(world: World) -> List[Node]:
    """Spawn models (drones and objects) of world."""
    models = world.drones + world.objects
    launch_processes = []
    for model in models:
        # ros2 run ros_gz_sim create -world ARG -file FILE
        gazebo_spawn_entity = Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=spawn_args(world, model)
        )
        launch_processes.append(gazebo_spawn_entity)

    return launch_processes


def launch_simulation(context: LaunchContext):
    """
    Return processes needed for launching the simulation.

    Simulator + Spawning Models + Bridges.
    """
    config_file = LaunchConfiguration('simulation_config_file').perform(context)

    # Check extension of config file
    if config_file.endswith('.json'):
        with open(config_file, 'r', encoding='utf-8') as stream:
            config = json.load(stream)
    elif config_file.endswith('.yaml') or config_file.endswith('.yml'):
        with open(config_file, 'r', encoding='utf-8') as stream:
            config = yaml.safe_load(stream)
    else:
        raise InvalidSimulationConfigFile('Invalid configuration file extension.')
    world = World(**config)

    launch_processes = []
    launch_processes.extend(spawn(world))
    return launch_processes


def generate_launch_description():
    """Generate Launch description with GzSim launch + Models Spawning + World/Object bridges."""
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'simulation_config_file',
            description='Launch config file (JSON or YAML format).'),
        # Launch processes
        OpaqueFunction(function=launch_simulation),
    ])
