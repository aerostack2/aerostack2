import os
import logging
from typing import List
from xml.etree import ElementTree
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

FORMAT = '[%(levelname)s] [launch]: %(message)s'
logging.basicConfig(format=FORMAT)

def get_available_plugins(package_name: str) -> List[str]:
    """Parse plugins.xml file from package and return a list of plugins from a specific type"""
    plugins_file = os.path.join(
        get_package_share_directory(package_name),
        'plugins.xml'
    )
    root = ElementTree.parse(plugins_file).getroot()

    available_plugins = []

    # Check if the root element is a <class_libraries> or <library> tag
    if root.tag == 'class_libraries':
        # Find all elements with the tag 'library' under the root
        libraries = root.findall('library')
    elif root.tag == 'library':
        # If the root is a single <library> tag, consider it as a list itself
        libraries = [root]
    else:
        # If the root tag is neither <class_libraries> nor <library>, return empty list
        return available_plugins

    for library in libraries:
        # Extract plugin information from the 'class' tag
        classes = library.findall('class')
        for plugin_class in classes:
            plugin_type = plugin_class.attrib.get('type')
            if plugin_type:
                plugin_name = plugin_type.split('::')[0]
                available_plugins.append(plugin_name)

    return available_plugins

def generate_launch_description() -> LaunchDescription:
    """Generate launch description with the controller node as composable."""
    
    declare_namespace = DeclareLaunchArgument(
        'namespace', description='Drone namespace'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false'
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace)
    ld.add_action(declare_use_sim_time)

    ld.add_action(
        DeclareLaunchArgument('plugin_name', description='Plugin name',
                              choices=get_available_plugins('as2_motion_controller'))
    )
    ld.add_action(
        DeclareLaunchArgument('plugin_config_file', default_value='',
                              description='Path to plugin config file')
    )
    ld.add_action(
        DeclareLaunchArgument('plugin_available_modes_config_file', default_value='',
                              description='Path to plugin available modes config file')
    )
    ld.add_action(
        DeclareLaunchArgument('motion_controller_config_file', default_value='',
                              description='Path to motion controller config file')
    )

    plugin_config_file = PathJoinSubstitution([
        FindPackageShare('as2_motion_controller'),
        'plugins', LaunchConfiguration('plugin_name'), 'config', 'controller_default.yaml'
    ])

    motion_controller_config_file = PathJoinSubstitution([
        FindPackageShare('as2_motion_controller'),
        'config', 'motion_controller_default.yaml'
    ])

    controller_node = ComposableNode(
        package='as2_motion_controller',
        plugin='controller_manager::ControllerManager',
        name='ControllerManager',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'plugin_name': LaunchConfiguration('plugin_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'plugin_available_modes_config_file': LaunchConfiguration('plugin_available_modes_config_file'),
            'plugin_config_file': plugin_config_file,
            'motion_controller_config_file': motion_controller_config_file
        }],
        output='screen',
        emulate_tty=True
    )

    container = ComposableNodeContainer(
        name='controller_container',
        namespace=LaunchConfiguration('namespace'),
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[controller_node],
        output='screen',
    )
    ld.add_action(container)
    return ld

