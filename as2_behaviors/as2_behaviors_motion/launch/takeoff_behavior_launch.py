""" Launch file for the takeoff behavior """
import sys
import logging
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

FORMAT = '[%(levelname)s] [launch]: %(message)s'
logging.basicConfig(format=FORMAT)


def get_node(context):
    """ Returns the takeoff behavior node """
    plugin_name = LaunchConfiguration('plugin_name').perform(context)
    if not plugin_name:
        logging.critical("Plugin not set.")
        sys.exit(1)

    parameters = [{
        'plugin_name': plugin_name,
        'takeoff_height': LaunchConfiguration('takeoff_height'),
        'takeoff_speed': LaunchConfiguration('takeoff_speed'),
        'takeoff_threshold': LaunchConfiguration('takeoff_threshold'),
    }]

    node = Node(
        package='as2_behaviors_motion',
        executable='takeoff_behavior_node',
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
        DeclareLaunchArgument('takeoff_height'),
        DeclareLaunchArgument('takeoff_speed'),
        DeclareLaunchArgument('takeoff_threshold'),
        OpaqueFunction(function=get_node)
    ])

    return launch_description
