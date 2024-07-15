"""
path_plannner.launch.py
"""
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable


def generate_launch_description():
    """entrypoint
    """
    return LaunchDescription([
        DeclareLaunchArgument('namespace', description="Drone namespace",
                              default_value=EnvironmentVariable('AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgument(
            'use_sim_time', description="Use sim time flag", default_value='false'),
        DeclareLaunchArgument(
            'use_path_optimizer', description="Use path optimizer", default_value='false'),
        DeclareLaunchArgument(
            'safety_distance', description="Safety distance to obstacles (drone size)",
            default_value='1.0'),
        DeclareLaunchArgument(
            'config_file', description="Path to config file. " +
            "Be careful, parameters in file will override launch arguments",
            default_value=''),
        Node(
            package="as2_path_planning",
            executable="path_planner",
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time'),
                 'use_path_optimizer': LaunchConfiguration('use_path_optimizer'),
                 'safety_distance': LaunchConfiguration('safety_distance')},
                LaunchConfiguration('config_file')
            ],
            output="screen",
            emulate_tty=True,
            # arguments=['--ros-args', '--log-level', 'DEBUG']
        ),
    ])
