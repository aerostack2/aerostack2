""" Launch file for the follow_path behaviour """
import sys
import logging
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

FORMAT = '[%(levelname)s] [launch]: %(message)s'
logging.basicConfig(format=FORMAT)


def get_controller_manager_node(context):
    """ Returns the follow_path behavior node """
    plugin_name = LaunchConfiguration('plugin_name').perform(context)
    if not plugin_name:
        logging.critical("Plugin not set.")
        sys.exit(1)

    parameters = [{
        'plugin_name': plugin_name,
        'follow_path_speed': LaunchConfiguration('follow_path_speed'),
        'follow_path_threshold': LaunchConfiguration('follow_path_threshold'),
    }]

    node = Node(
        package='follow_path_behaviour',
        executable='follow_path_behaviour_node',
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
        DeclareLaunchArgument('follow_path_speed'),
        DeclareLaunchArgument('follow_path_threshold'),
        OpaqueFunction(function=get_controller_manager_node)
    ])

    return launch_description
