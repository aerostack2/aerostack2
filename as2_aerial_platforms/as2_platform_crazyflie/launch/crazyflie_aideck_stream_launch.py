"""Launch Crazyflie Aideck platform node"""
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch import LaunchDescription


def generate_launch_description():
    """Entrypoint"""

    aideck_config_file = PathJoinSubstitution([
        FindPackageShare('as2_platform_crazyflie'),
        'config', 'aideck_config_file.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('namespace',
                              default_value=EnvironmentVariable(
                                  'AEROSTACK2_SIMULATION_DRONE_ID'),
                              description='Drone namespace'),
        DeclareLaunchArgument('aideck_config_file',
                              default_value=aideck_config_file,
                              description='Aideck configuration file'),

        Node(
            package="as2_platform_crazyflie",
            executable="aideck_node.py",
            name="aideck_pub",
            namespace=LaunchConfiguration('namespace'),
            output="screen",
            emulate_tty=True,
            parameters=[
                LaunchConfiguration('aideck_config_file')
            ],
        )
    ])
