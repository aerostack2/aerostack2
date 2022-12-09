from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import yaml
import logging
FORMAT = '[%(levelname)s] [launch]: %(message)s'
logging.basicConfig(format=FORMAT)

def get_controller_node(context, *args, **kwargs):
    config = LaunchConfiguration('config').perform(context)

    with open(config, "r") as f:
        config_params = yaml.safe_load(f)

    try:
        plugin_name = config_params["/**"]["ros__parameters"]["plugin_name"]
    except KeyError:
        plugin_name = ""

    if not plugin_name:
        logging.critical("Plugin not set.")
        exit(-1)

    try:
        plugin_config = config_params["/**"]["ros__parameters"]["plugin_config_file"]
    except KeyError:
        plugin_config = ""

    if not plugin_config:
        plugin_config = PathJoinSubstitution([
            FindPackageShare(plugin_name),
            'config', 'default_controller.yaml'
        ])

    node = Node(
        package='controller_manager',
        executable='controller_manager_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[LaunchConfiguration('config'), 
                    plugin_config, 
                    {"use_sim_time": LaunchConfiguration('use_sim_time')},
                    {"odom_frame_id": LaunchConfiguration('odom_frame_id')},
                    {"base_frame_id": LaunchConfiguration('base_frame_id')},
                    {"use_bypass": LaunchConfiguration('use_bypass')}],
        output='screen',
        emulate_tty=True
    )

    return [node]


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('controller_manager'),
        'config', 'controller_manager.yaml'
    ])

    ld = LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=EnvironmentVariable('AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('odom_frame_id', default_value='odom'),
        DeclareLaunchArgument('base_frame_id', default_value='base_link'),
        DeclareLaunchArgument('use_bypass', default_value='true'),
        DeclareLaunchArgument('config', default_value=config),
        OpaqueFunction(function=get_controller_node)
    ])

    return ld
