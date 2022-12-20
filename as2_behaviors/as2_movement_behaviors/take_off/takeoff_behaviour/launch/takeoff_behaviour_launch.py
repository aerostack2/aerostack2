""" Launch file for the takeoff behaviour """
import sys
import logging
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

FORMAT = '[%(levelname)s] [launch]: %(message)s'
logging.basicConfig(format=FORMAT)


def get_controller_manager_node(context):
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
        package='takeoff_behaviour',
        executable='takeoff_behaviour_node',
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
        DeclareLaunchArgument('plugin_name', default_value=''),
        DeclareLaunchArgument('takeoff_height', default_value=1.0),
        DeclareLaunchArgument('takeoff_speed', default_value=0.2),
        DeclareLaunchArgument('takeoff_threshold', default_value=0.1),
        OpaqueFunction(function=get_controller_manager_node)
    ])

    return launch_description
