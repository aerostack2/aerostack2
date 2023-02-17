""" Launch file for the go_to behavior """
import sys
import logging
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

FORMAT = '[%(levelname)s] [launch]: %(message)s'
logging.basicConfig(format=FORMAT)


def get_node(context):
    """ Returns the go_to behavior node """
    plugin_name = LaunchConfiguration('plugin_name').perform(context)
    if not plugin_name:
        logging.critical("Plugin not set.")
        sys.exit(1)

    parameters = [{
        'plugin_name': plugin_name,
        'go_to_speed': LaunchConfiguration('go_to_speed'),
        'go_to_threshold': LaunchConfiguration('go_to_threshold'),
    }]

    node = Node(
        package='as2_behaviors_motion',
        executable='go_to_behavior_node',
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
        DeclareLaunchArgument('go_to_speed'),
        DeclareLaunchArgument('go_to_threshold'),
        OpaqueFunction(function=get_node)
    ])

    return launch_description
