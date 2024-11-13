#!/bin/python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from as2_core.declare_launch_arguments_from_config_file import DeclareLaunchArgumentsFromConfigFile
from as2_core.launch_configuration_from_config_file import LaunchConfigurationFromConfigFile
from ament_index_python.packages import get_package_share_directory
from as2_core.declare_launch_arguments_from_config_file import DeclareLaunchArgumentsFromConfigFile
from as2_core.launch_configuration_from_config_file import LaunchConfigurationFromConfigFile


def generate_launch_description():
    # package_folder = get_package_share_directory(
    #     'gates_to_waypoints')
    # config_file = os.path.join(package_folder, 'config/config.yaml')
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
        DeclareLaunchArgument('spin_rate',
                              description='Spin rate',
                              default_value='20'),
        DeclareLaunchArgument('mission_path',
                              description='Path where the mission is located'),
        DeclareLaunchArgument('mission_name',
                              description='Name of the mission to be executed'),
        # DeclareLaunchArgumentsFromConfigFile('config_file',
        #                                      config_file,
        #                                      description='Configuration file'),
        Node(
            package='as2_python_api',
            namespace=LaunchConfiguration('namespace'),
            executable='mission_runner',
            output='screen',
            arguments=['--use_sim_time', LaunchConfiguration('use_sim_time'),
                       '--ns', LaunchConfiguration('namespace'),
                       '--spin-rate', LaunchConfiguration('spin_rate'),
                       '--log-level', LaunchConfiguration('log_level')],
            emulate_tty=True,
            parameters=[
                {
                    'mission_path': LaunchConfiguration('mission_path'),
                    'mission_name': LaunchConfiguration('mission_name'),
                },
                # LaunchConfigurationFromConfigFile(
                #     'config_file',
                #     default_file=config_file)
            ]
        ),
    ])
