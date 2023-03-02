""" Launch file for the land behavior """
import sys
import logging
import os
from xml.etree import ElementTree
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

FORMAT = '[%(levelname)s] [launch]: %(message)s'
logging.basicConfig(format=FORMAT)


def get_available_plugins(package_name: str, plugin_type: str) -> list[str]:
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
    """ Returns the land behavior node """
    plugin_name = LaunchConfiguration('plugin_name').perform(context)
    if not plugin_name:
        logging.critical("Plugin not set.")
        sys.exit(1)

    parameters = {
        'plugin_name': plugin_name,
        'land_speed': LaunchConfiguration('land_speed'),
    }

    if 'speed' in plugin_name:
        parameters['land_speed_condition_percentage'] = LaunchConfiguration(
            'land_speed_condition_percentage')
        parameters['land_speed_condition_height'] = LaunchConfiguration(
            'land_speed_condition_height')

    node = Node(
        package='as2_behaviors_motion',
        executable='land_behavior_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[parameters],
        output='screen',
        emulate_tty=True
    )

    return [node]


def generate_launch_description():
    """ Returns the launch description """
    launch_description = LaunchDescription([
        DeclareLaunchArgument('namespace', description='Drone namespace'),
        DeclareLaunchArgument('plugin_name', description='Land plugin name',
                              choices=get_available_plugins('as2_behaviors_motion', 'land')),
        DeclareLaunchArgument('land_speed', description='Default land speed'),
        DeclareLaunchArgument(
            'land_speed_condition_percentage', default_value='0.2',
            description='Speed condition to finish land. Only used with land_plugin_speed'),
        DeclareLaunchArgument(
            'land_speed_condition_height', default_value='0.2',
            description='Height condition to finish land. Only used with land_plugin_speed'),
        OpaqueFunction(function=get_node)
    ])

    return launch_description
