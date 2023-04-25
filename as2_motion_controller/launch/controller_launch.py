""" Launch file for the controller manager node """

import os
import sys
import logging
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from xml.etree import ElementTree

FORMAT = '[%(levelname)s] [launch]: %(message)s'
logging.basicConfig(format=FORMAT)


def get_available_plugins(package_name: str) -> list[str]:
    """
    Parse plugins.xml file from package and return a list of plugins from a specific type
    """
    plugins_file = os.path.join(
        get_package_share_directory(package_name),
        'plugins.xml'
    )
    root = ElementTree.parse(plugins_file).getroot()

    available_plugins = []
    for class_element in root.findall('class'):
        available_plugins.append(
            class_element.attrib['type'].split('::')[0])
    return available_plugins


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
        print("Finding default config file for plugin: " + plugin_name)
        plugin_config_file = PathJoinSubstitution([
            FindPackageShare('as2_motion_controller'),
            'plugins/' + plugin_name + '/config', 'default_controller.yaml'
        ])
        print("Found default config file: plugins/" +
              plugin_name + "/config/default_controller.yaml")

    parameters.append(plugin_config_file)

    node = Node(
        package='as2_motion_controller',
        executable='as2_motion_controller_node',
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
        DeclareLaunchArgument(
            'plugin_name', choices=get_available_plugins('as2_motion_controller')),
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
