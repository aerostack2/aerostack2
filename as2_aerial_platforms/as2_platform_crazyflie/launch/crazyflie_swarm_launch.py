"""Launch Crazyflie Swarm platform node"""
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch import LaunchDescription


def generate_launch_description():
    """Entrypoint"""

    control_modes = PathJoinSubstitution([
        FindPackageShare('as2_platform_crazyflie'),
        'config', 'control_modes.yaml'
    ])

    platform_config_file = PathJoinSubstitution([
        FindPackageShare('as2_platform_crazyflie'),
        'config', 'platform_config_file.yaml'
    ])

    swarm_config_file = PathJoinSubstitution([
        FindPackageShare('as2_platform_crazyflie'),
        'config', 'swarm_config_file.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('control_modes_file',
                              default_value=control_modes,
                              description='Platform control modes file'),
        DeclareLaunchArgument('platform_config_file',
                              default_value=platform_config_file,
                              description='Platform configuration file'),
        DeclareLaunchArgument('swarm_config_file',
                              default_value=swarm_config_file,
                              description='Platform swarm URI configuration file'),

        Node(
            package="as2_platform_crazyflie",
            executable="as2_platform_crazyflie_swarm_node",
            name="platform",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "control_modes_file": LaunchConfiguration('control_modes_file'),
                    "swarm_config_file": LaunchConfiguration('swarm_config_file')
                },
                LaunchConfiguration('platform_config_file')
            ],
        )
    ])
