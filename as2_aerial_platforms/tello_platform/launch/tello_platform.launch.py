from os.path import join

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = join(
        get_package_share_directory('tello_platform'),
        'config',
        'control_modes.yaml'
    )
    DRONE_ID = os.environ['AEROSTACK2_SIMULATION_DRONE_ID']
    return LaunchDescription([
        DeclareLaunchArgument('drone_id',
                              default_value=DRONE_ID,
                              description='Drone namespace.'),
        DeclareLaunchArgument('mass', default_value='0.029'),
        DeclareLaunchArgument('max_thrust', default_value='0.0'),
        DeclareLaunchArgument('min_thrust', default_value='0.0'),
        DeclareLaunchArgument('cmd_freq', default_value='100.0'),
        DeclareLaunchArgument('info_freq', default_value='10.0'),
        DeclareLaunchArgument('sensor_freq', default_value='10.0'),
        DeclareLaunchArgument('control_modes_file', default_value=config),
        # DeclareLaunchArgument('drone_URI',
        #                       default_value='radio://0/80/2M/E7E7E7E7E7',
        #                       description='Crazyflie URI.'),
        DeclareLaunchArgument('external_odom', default_value='false',
                              choices=["true", "false"],
                              description='Availability of external odometry'),
        DeclareLaunchArgument('external_odom_topic',
                              default_value='external_odom',
                              description='External odometry topic name.'),
        DeclareLaunchArgument('simulation_mode', default_value='false',
                              choices=["true", "false"],
                              description="Simulation flag."),
        # DeclareLaunchArgument('ip', default_value="192.168.43.95"),
        # DeclareLaunchArgument('port', default_value='5000'),
        # DeclareLaunchArgument('save_flag', default_value='False'),
        # DeclareLaunchArgument('show_flag', default_value='False'),

        Node(
            package="tello_platform",
            executable="tello_platform_node",
            name="platform",
            namespace=LaunchConfiguration('drone_id'),
            output="screen",
            emulate_tty=True,
            parameters=[{
                "simulation_mode": LaunchConfiguration('simulation_mode'),
                "mass": LaunchConfiguration('mass'),
                "max_thrust": LaunchConfiguration('max_thrust'),
                "min_thrust": LaunchConfiguration('min_thrust'),
                "cmd_freq": LaunchConfiguration('cmd_freq'),
                "info_freq": LaunchConfiguration('info_freq'),
                "sensor_freq": LaunchConfiguration('sensor_freq'),
                "control_modes_file":
                    LaunchConfiguration('control_modes_file'),
                "external_odom": LaunchConfiguration('external_odom'),
                # "drone_URI": LaunchConfiguration('drone_URI'),
                "external_odom_topic":
                    LaunchConfiguration('external_odom_topic'),
            }],
        )
    ])
