""" Launch file for the state estimator node """

import os
import sys
import logging
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution


FORMAT = '[%(levelname)s] [launch]: %(message)s'
logging.basicConfig(format=FORMAT)


def get_state_estimator_node(context):
    """ Returns the state estimator node """
    plugin_name = LaunchConfiguration('plugin_name').perform(context)
    if not plugin_name:
        logging.critical("Plugin not set.")
        sys.exit(1)

    plugin_config_file = LaunchConfiguration(
        'plugin_config_file').perform(context)

    parameters = [{
        'plugin_name': plugin_name,
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'base_frame': LaunchConfiguration('base_frame'),
        'global_ref_frame': LaunchConfiguration(
            'global_ref_frame'),
        'odom_frame': LaunchConfiguration('odom_frame'),
        'map_frame': LaunchConfiguration('map_frame'),
    }]

    if not plugin_config_file:
        try:
            plugin_config_file = PathJoinSubstitution([
                FindPackageShare(plugin_name),
                'config', 'default_state_estimator.yaml'
            ])
            plugin_config_file.perform(context)
        except Exception:
            plugin_config_file = PathJoinSubstitution([
                FindPackageShare('as2_state_estimator'),
                'plugins/' + plugin_name + '/config', 'default_state_estimator.yaml'
            ])
        
    
    parameters.append(plugin_config_file)

    node = Node(
        package='as2_state_estimator',
        executable='as2_state_estimator_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=parameters,
        output='screen',
        emulate_tty=True
    )

    return [node]


def generate_launch_description():
    """ Returns the launch description """
    launch_description = LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=EnvironmentVariable(
            'AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgument('plugin_name'),
        DeclareLaunchArgument('plugin_config_file', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('global_ref_frame', default_value='earth'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        OpaqueFunction(function=get_state_estimator_node)
    ])

    return launch_description
