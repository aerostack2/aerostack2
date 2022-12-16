""" Launch file for the land behaviour """
import sys
import logging
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

FORMAT = '[%(levelname)s] [launch]: %(message)s'
logging.basicConfig(format=FORMAT)


def get_controller_manager_node(context):
    """ Returns the land behavior node """
    plugin_name = LaunchConfiguration('plugin_name').perform(context)
    if not plugin_name:
        logging.critical("Plugin not set.")
        sys.exit(1)

    parameters = [{
        'plugin_name': plugin_name,
        'land_speed': LaunchConfiguration('land_speed'),
    }]

    node = Node(
        package='land_behaviour',
        executable='land_behaviour_node',
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
        DeclareLaunchArgument('land_speed'),
        OpaqueFunction(function=get_controller_manager_node)
    ])

    return launch_description
