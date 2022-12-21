from os.path import join

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = join(
        get_package_share_directory('as2_tello_platform'),
        'config',
        'control_modes.yaml'
    )
    DRONE_ID = os.environ['AEROSTACK2_SIMULATION_DRONE_ID']
    return LaunchDescription([
        DeclareLaunchArgument('drone_id',
                              default_value=DRONE_ID,
                              description='Drone namespace.'),
        DeclareLaunchArgument('cmd_freq', default_value='10.0'),
        DeclareLaunchArgument('info_freq', default_value='10.0'),
        DeclareLaunchArgument('sensor_freq', default_value='10.0'),
        DeclareLaunchArgument('control_modes_file', default_value=config),
        DeclareLaunchArgument('simulation_mode', default_value='false',
                              choices=["true", "false"],
                              description="Simulation flag."),

        Node(
            package="as2_tello_platform",
            executable="as2_tello_platform_node",
            name="platform",
            namespace=LaunchConfiguration('drone_id'),
            output="screen",
            emulate_tty=True,
            parameters=[{
                "simulation_mode": LaunchConfiguration('simulation_mode'),
                "cmd_freq": LaunchConfiguration('cmd_freq'),
                "info_freq": LaunchConfiguration('info_freq'),
                "sensor_freq": LaunchConfiguration('sensor_freq'),
                "control_modes_file":
                    LaunchConfiguration('control_modes_file'),
            }],
        )
    ])
