""" Launch file for the state estimator node """

from __future__ import annotations
import sys
import os
import logging
from xml.etree import ElementTree

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LoadComposableNodes, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from ament_index_python.packages import PackageNotFoundError

FORMAT = '[%(levelname)s] [launch]: %(message)s'
logging.basicConfig(format=FORMAT)

def get_available_plugins(package_name: str) -> list[str]:
    """Parse plugins.xml file from package and return a list of plugins from a specific type."""
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

def get_state_estimator_node(context):
    """ Returns the state estimator node """
    plugin_name = LaunchConfiguration('plugin_name').perform(context)
    # plugin_name += "::Plugin"
    print(plugin_name)
    if not plugin_name:
        logging.critical("Plugin not set.")
        sys.exit(1)

    plugin_config_file = LaunchConfiguration(
        'plugin_config_file').perform(context)

    parameters = [{
        'plugin_name': plugin_name,
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'base_frame': LaunchConfiguration('base_frame'),
        'global_ref_frame': LaunchConfiguration('global_ref_frame'),
        'odom_frame': LaunchConfiguration('odom_frame'),
        'map_frame': LaunchConfiguration('map_frame'),
        'rigid_body_name': LaunchConfiguration('rigid_body_name'),
        'mocap_topic': LaunchConfiguration('mocap_topic'),
        'twist_smooth_filter_cte': LaunchConfiguration('twist_smooth_filter_cte'),
        'orientation_smooth_filter_cte': LaunchConfiguration('orientation_smooth_filter_cte'),
    }]

    if not plugin_config_file:
        try:
            plugin_config_file = PathJoinSubstitution([
                FindPackageShare(plugin_name),
                'config', 'default_state_estimator.yaml'
            ])
            plugin_config_file.perform(context)
        except PackageNotFoundError:
            plugin_config_file = PathJoinSubstitution([
                FindPackageShare('as2_state_estimator'),
                'plugins/' + plugin_name + '/config', 'default_state_estimator.yaml'
            ])

    parameters.append(plugin_config_file)

    node = ComposableNode(
        package='as2_state_estimator',
        plugin = 'StateEstimator',
        namespace=LaunchConfiguration('namespace'),
        parameters=parameters,
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    container = ComposableNodeContainer(
        # condition=LaunchConfigurationEquals('container', ''),
        name=LaunchConfiguration('container'),
        namespace=LaunchConfiguration('namespace'),
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            node
        ],
        output='screen'
    )
    return [container]


def generate_launch_description():
    """ Returns the launch description """
    launch_description = LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=EnvironmentVariable(
            'AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgument('plugin_name'),
        DeclareLaunchArgument('plugin_config_file', default_value=get_available_plugins('as2_state_estimator')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('global_ref_frame', default_value='earth'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('rigid_body_name', default_value=''),
        DeclareLaunchArgument('mocap_topic', default_value=''),
        DeclareLaunchArgument(
            'twist_smooth_filter_cte', default_value='0.1',
            description='Smoothing constant for the twist filter. ' +
            'The closer to 0, the smoother the output, while the closer to 1, ' +
            'the more responsive the output (1 is equivalent to no smoothing). ' +
            'Only used in the mocap plugin.'),
        DeclareLaunchArgument(
            'orientation_smooth_filter_cte', default_value='1.0',
            description='Smoothing constant for the orientation filter. ' +
            'The closer to 0, the smoother the output, while the closer to 1, ' +
            'the more responsive the output (1 is equivalent to no smoothing). ' +
            'Only used in the mocap plugin.'),
        OpaqueFunction(function=get_state_estimator_node)
    ])

    return launch_description
