""" Launch file for the controller manager node """

import os
import sys
import logging
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

FORMAT = '[%(levelname)s] [launch]: %(message)s'
logging.basicConfig(format=FORMAT)


def get_controller_manager_node(context):
    """ Returns the controller manager node """
    plugin_name = LaunchConfiguration('plugin_name').perform(context)
    if not plugin_name:
        logging.critical("Plugin not set.")
        sys.exit(1)

    parameters = [{
        'plugin_name': plugin_name,
        'plugin_available_modes_config_file': LaunchConfiguration(
            'plugin_available_modes_config_file'),
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'cmd_freq': LaunchConfiguration('cmd_freq'),
        'info_freq': LaunchConfiguration('info_freq'),
        'odom_frame_id': LaunchConfiguration('odom_frame_id'),
        'base_frame_id': LaunchConfiguration('base_frame_id'),
        'use_bypass': LaunchConfiguration('use_bypass'),
    }]

    plugin_config_file = LaunchConfiguration(
        'plugin_config_file').perform(context)

    if not plugin_config_file:
        plugin_config_file = PathJoinSubstitution([
            FindPackageShare('as2_' + plugin_name),
            'config', 'default_controller.yaml'
        ])
        
    parameters.append(plugin_config_file)

    node = Node(
        package='as2_controller_manager',
        executable='as2_controller_manager_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=parameters,
        output='screen',
        emulate_tty=True
    )

    return [node]


def generate_launch_description():
    """ Returns the launch description """
    launch_description = LaunchDescription([
        DeclareLaunchArgument('namespace'),
        DeclareLaunchArgument('plugin_name'),
        DeclareLaunchArgument('plugin_config_file', default_value=''),
        DeclareLaunchArgument(
            'plugin_available_modes_config_file', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('cmd_freq', default_value='100'),
        DeclareLaunchArgument('info_freq', default_value='10'),
        DeclareLaunchArgument('odom_frame_id', default_value='odom'),
        DeclareLaunchArgument('base_frame_id', default_value='base_link'),
        DeclareLaunchArgument('use_bypass', default_value='true'),
        OpaqueFunction(function=get_controller_manager_node)
    ])

    return launch_description
