""" Launch file for the state estimator node """

import os
import sys
import logging
from launch_ros.actions import LoadComposableNodes, Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
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

    # node = Node(
    #     package='as2_state_estimator',
    #     executable='as2_state_estimator_node',
    #     namespace=LaunchConfiguration('namespace'),
    #     parameters=parameters,
    #     output='screen',
    #     emulate_tty=True
    # )

    return []


def generate_launch_description():
    """ Returns the launch description """
    plugin_config_file = PathJoinSubstitution([
                FindPackageShare('as2_state_estimator'),
                'plugins/' + 'StateEstimator' + '/config', 'state_estimator_config_file.yaml'
            ])
    
    container = ComposableNodeContainer(
        name='container',   # Nombre container donde se indexa
        namespace='drone0',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='as2_state_estimator',
                plugin='StateEstimatorBase',
                name='StateEstimator',
                namespace='drone0',
                parameters=[{
                    'namespace':'drone0',
                    'plugin_name':'ground_truth',
                    'use_sim_time':True,
                    'plugin_config_file':plugin_config_file,
                    'base_frame':'base_link',
                    'global_ref_frame':'earth',
                    'odom_frame':'odom',
                    'map_frame':'map'
            }])
        ],
        output='both',
    )

    # container = ComposableNodeContainer(
    #     name='hola',
    #     namespace='drone0',
    #     package='rclcpp_components',
    #     executable='component_container',
    #     composable_node_descriptions=[cnode],
    #     output='screen',
    # )

    launch_description = LaunchDescription([container])

    return launch_description
