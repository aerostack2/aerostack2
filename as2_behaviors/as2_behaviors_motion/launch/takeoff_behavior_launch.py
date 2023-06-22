""" Launch file for the motion behavior """

import os
import sys
import logging
from typing import List
from xml.etree import ElementTree
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

FORMAT = '[%(levelname)s] [launch]: %(message)s'
logging.basicConfig(format=FORMAT)

BEHAVIOR_NAME = 'takeoff'


def get_available_plugins(package_name: str, plugin_type: str) -> List[str]:
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
        if plugin_type in class_element.attrib['type']:
            available_plugins.append(
                class_element.attrib['type'].split('::')[0])
    return available_plugins


def get_node(context):
    """ Returns the behavior node """
    plugin_name = LaunchConfiguration('plugin_name').perform(context)
    if not plugin_name:
        logging.critical("Plugin not set.")
        sys.exit(1)

    parameters = [
        {"use_sim_time": LaunchConfiguration('use_sim_time')},
        {'plugin_name': plugin_name}
    ]

    behavior_config_file = LaunchConfiguration(
        'behavior_config_file').perform(context)

    if not behavior_config_file:
        behavior_config_file = PathJoinSubstitution([
            FindPackageShare('as2_behaviors_motion'),
            'config/' + BEHAVIOR_NAME + '_behavior/config_default.yaml'
        ])

    parameters.append(behavior_config_file)

    node = Node(
        package='as2_behaviors_motion',
        executable=BEHAVIOR_NAME + '_behavior_node',
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
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('plugin_name', description='Plugin name',
                              choices=get_available_plugins('as2_behaviors_motion', BEHAVIOR_NAME)),
        DeclareLaunchArgument('behavior_config_file', default_value='',
                              description='Path to behavior config file'),
        OpaqueFunction(function=get_node)
    ])

    return launch_description
