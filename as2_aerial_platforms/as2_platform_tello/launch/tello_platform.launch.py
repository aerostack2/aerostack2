"""Launch DJI Tello platform node"""
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch import LaunchDescription


def generate_launch_description():
    """Entrypoint"""

    control_modes = PathJoinSubstitution([
        FindPackageShare('as2_platform_tello'),
        'config', 'control_modes.yaml'
    ])

    platform_config_file = PathJoinSubstitution([
        FindPackageShare('as2_platform_tello'),
        'config', 'platform_default.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('namespace',
                              default_value=EnvironmentVariable(
                                  'AEROSTACK2_SIMULATION_DRONE_ID'),
                              description='Drone namespace'),
        DeclareLaunchArgument('control_modes_file',
                              default_value=control_modes,
                              description='Platform control modes file'),
        DeclareLaunchArgument('platform_config_file',
                              default_value=platform_config_file,
                              description='Platform configuration file'),

        Node(
            package="as2_platform_tello",
            executable="as2_platform_tello_node",
            name="platform",
            namespace=LaunchConfiguration('namespace'),
            output="screen",
            emulate_tty=True,
            parameters=[{
                    "control_modes_file": LaunchConfiguration('control_modes_file'),
            },
                LaunchConfiguration('platform_config_file')
            ],
        )
    ])
