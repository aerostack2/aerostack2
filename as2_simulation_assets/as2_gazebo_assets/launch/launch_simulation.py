"""
launch_simulation.py
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


__authors__ = "Pedro Arias Pérez, Javier Melero Deza, Rafael Pérez Seguí"
__copyright__ = "Copyright (c) 2022 Universidad Politécnica de Madrid"
__license__ = "BSD-3-Clause"
__version__ = "0.1.0"

import os
import json
from typing import List
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, \
    RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, EmitEvent
from launch.events import Shutdown

from as2_gazebo_assets.world import World, spawn_args


def simulation(world_name: str, gui_config: str = '', headless: bool = False,
               verbose: bool = False, run_on_start: bool = True):
    """Open Gazebo simulator
    """
    gz_args = []
    if gui_config != '':
        gz_args.append(f'--gui-config {gui_config}')
    if verbose:
        gz_args.append('-v 4')
    if run_on_start:
        gz_args.append('-r')
    if headless:
        gz_args.append('-s')

    if world_name.split('.')[-1] == 'sdf':
        gz_args.append(world_name)
    else:
        gz_args.append(f'{world_name}.sdf')

    # ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="empty.sdf"
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'),
            '/gz_sim.launch.py']),
        launch_arguments={'gz_args': ' '.join(gz_args)}.items())

    # Register handler for shutting down ros launch when gazebo process exits
    # monitor_sim.py will run until it can not find the gazebo process.
    # Once monitor_sim.py exits, a process exit event is triggered which causes the
    # handler to emit a Shutdown event

    path = os.path.join(get_package_share_directory('as2_gazebo_assets'), 'launch',
                        'monitor_sim.py')
    monitor_sim_proc = ExecuteProcess(
        cmd=['python3', path],
        name='monitor_sim',
        output='screen',
    )
    sim_exit_event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=monitor_sim_proc,
            on_exit=[
                EmitEvent(event=Shutdown(reason='Simulation ended'))
            ]
        )
    )

    # return [gz_sim]
    return [gz_sim, monitor_sim_proc, sim_exit_event_handler]


def spawn(world: World) -> List[Node]:
    """Spawn models (drones and objects) of world"""
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


def world_bridges():
    """Create world bridges. Mainly clock if sim_time enabled."""
    world_bridges_ = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('as2_gazebo_assets'), 'launch'),
            '/world_bridges.py']),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items(),
    )
    return [world_bridges_]


def object_bridges():
    """Create object bridges."""
    object_bridges_ = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('as2_gazebo_assets'), 'launch'),
            '/object_bridges.py']),
        launch_arguments={
            'simulation_config_file': LaunchConfiguration('simulation_config_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items(),
    )
    return [object_bridges_]


def launch_simulation(context: LaunchContext):
    """Return processes needed for launching the simulation.
    Simulator + Spawning Models + Bridges.
    """
    config_file = LaunchConfiguration(
        'simulation_config_file').perform(context)
    gui_config_file = LaunchConfiguration('gui_config_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time = use_sim_time.lower() in ['true', 't', 'yes', 'y', '1']
    headless = LaunchConfiguration('headless').perform(context)
    headless = headless.lower() in ['true', 't', 'yes', 'y', '1']
    verbose = LaunchConfiguration('verbose').perform(context)
    verbose = verbose.lower() in ['true', 't', 'yes', 'y', '1']
    run_on_start = LaunchConfiguration('run_on_start').perform(context)
    run_on_start = run_on_start.lower() in ['true', 't', 'yes', 'y', '1']

    with open(config_file, 'r', encoding='utf-8') as stream:
        config = json.load(stream)
        world = World(**config)

    launch_processes = []
    # If there is a world file created by jinja we use that one,
    # otherwise we use the default world model
    world_to_load = world.world_path if hasattr(
        world, 'world_path') else world.world_name
    launch_processes.extend(simulation(
        world_to_load, gui_config_file, headless, verbose, run_on_start))
    launch_processes.extend(spawn(world))
    launch_processes.extend(world_bridges() + object_bridges())
    return launch_processes


def generate_launch_description():
    """Generate Launch description with GzSim launch + Models Spawning + World/Object bridges
    """
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'simulation_config_file',
            description='Launch config file (JSON or YAML format).'),
        DeclareLaunchArgument(
            'gui_config_file',
            default_value='',
            description='GUI config file.'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
            description='Deactivates clock bridge and object publishes tf in sys clock time.'),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            choices=['true', 'false'],
            description='Launch in headless mode (only gz server).'),
        DeclareLaunchArgument(
            'verbose',
            default_value='false',
            choices=['true', 'false'],
            description='Launch in verbose mode.'),
        DeclareLaunchArgument(
            'run_on_start',
            default_value='true',
            choices=['true', 'false'],
            description='Run simulation on start.'),
        # Launch processes
        OpaqueFunction(function=launch_simulation),
    ])
