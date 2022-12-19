"""Keyboard Teleopration launch."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction


def launch_teleop(context):
    """Teleop python process."""
    keyboard_teleop = os.path.join(get_package_share_directory(
        'keyboard_teleoperation'), 'keyboard_teleoperation.py')

    drone_id = LaunchConfiguration('drone_id').perform(context)
    verbose = LaunchConfiguration('verbose').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    process = ExecuteProcess(
        cmd=['python3', keyboard_teleop, drone_id, verbose, use_sim_time],
        name='keyboard_teleoperation',
        output='screen')
    return [process]


def generate_launch_description():
    """Entrypoint launch description method."""
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'drone_id',
            description='Drone id.'),
        DeclareLaunchArgument(
            'verbose',
            default_value='false',
            choices=['true', 'false'],
            description='Launch in verbose mode.'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Use simulation time.'),
        OpaqueFunction(function=launch_teleop),
    ])
