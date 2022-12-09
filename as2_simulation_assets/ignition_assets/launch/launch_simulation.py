from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, EmitEvent
from launch_ros.actions import Node
from launch.events import Shutdown

from ign_assets.model import Model

import os
import json


def simulation(world_name, headless=False, verbose=False, run_on_start=True):
    ign_args = []
    if verbose:
        ign_args.append('-v 4')
    if run_on_start:
        ign_args.append('-r')
    if headless:
        ign_args.append('-s')

    if world_name.split('.')[-1] == 'sdf':
        ign_args.append(world_name)
    else:
        ign_args.append(f'{world_name}.sdf')

    # ros2 launch ros_gz_sim ign_gazebo.launch.py ign_args:="empty.sdf"
    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'),
            '/gz_sim.launch.py']),
        launch_arguments={'gz_args': ' '.join(ign_args)}.items())

    # Register handler for shutting down ros launch when ign gazebo process exits
    # monitor_sim.py will run until it can not find the ign gazebo process.
    # Once monitor_sim.py exits, a process exit event is triggered which causes the
    # handler to emit a Shutdown event
    p = os.path.join(get_package_share_directory('ignition_assets'), 'launch',
                     'monitor_sim.py')
    monitor_sim_proc = ExecuteProcess(
        cmd=['python3', p],
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

    return [ign_gazebo]    
    return [ign_gazebo, monitor_sim_proc, sim_exit_event_handler]    


def spawn(world_name, models):
    if type(models) != list:
        models = [models]

    # ros2 run ros_gz_sim create -world ARG -file FILE 
    launch_processes = []
    for model in models:
        ignition_spawn_entity = Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=model.spawn_args(world_name)
        )
        launch_processes.append(ignition_spawn_entity)

    return launch_processes

def world_bridges():
    world_bridges = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('ignition_assets'), 'launch'),
        '/world_bridges.py']))
    return [world_bridges]

def launch_simulation(context, *args, **kwargs):
    config_file = LaunchConfiguration('config_file').perform(context)
    headless = LaunchConfiguration('headless').perform(context)
    headless = headless.lower() in ['true', 't', 'yes', 'y', '1']
    verbose = LaunchConfiguration('verbose').perform(context)
    verbose = verbose.lower() in ['true', 't', 'yes', 'y', '1']
    run_on_start = LaunchConfiguration('run_on_start').perform(context)
    run_on_start = run_on_start.lower() in ['true', 't', 'yes', 'y', '1']

    with open(config_file, 'r') as stream:
        config = json.load(stream)
        if 'world' not in config:
            raise RuntimeError('Cannot construct bridges without world in config')
        world_name = config['world']

    with open(config_file, 'r') as stream:
        models = Model.FromConfig(stream)

    launch_processes = []

    launch_processes.extend(simulation(world_name, headless, verbose, run_on_start))
    launch_processes.extend(spawn(world_name, models))
    launch_processes.extend(world_bridges())
    return launch_processes


def generate_launch_description():
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'config_file',
            description='Launch config file (JSON or YAML format).'),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            choices= ['true', 'false'],
            description='Launch in headless mode (only ign server).'),
        DeclareLaunchArgument(
            'verbose',
            default_value='false',
            choices= ['true', 'false'],
            description='Launch in verbose mode.'),
        DeclareLaunchArgument(
            'run_on_start',
            default_value='true',
            choices= ['true', 'false'],
            description='Run simulation on start.'),
        OpaqueFunction(function=launch_simulation),
    ])
