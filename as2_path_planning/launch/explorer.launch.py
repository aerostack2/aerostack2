"""
explorer.py
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
            'safety_distance', description="Safety distance to obstacles (drone size)",
            default_value='1.0'),
        DeclareLaunchArgument(
            'reached_dist_thresh', description="Threshold to consider point as reached",
            default_value='0.5'),
        DeclareLaunchArgument(
            'spin_yaw_thresh', description="Threshold to spinning as finished",
            default_value='0.05'),
        DeclareLaunchArgument(
            'navigation_speed', description="Cruise speed of the drone during navigation",
            default_value='1.0'),
        DeclareLaunchArgument(
            'cautiously', description="Spin before moving to frontiers",
            default_value='False'),
        DeclareLaunchArgument(
            'spin_speed', description="Spin speed (rad/s) on cautious exploration mode",
            default_value='0.15'),
        DeclareLaunchArgument(
            'config_file', description="Path to config file. " +
            "Be careful, parameters in file will override launch arguments",
            default_value=''),
        Node(
            package="as2_path_planning",
            executable="explorer",
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time'),
                 'safety_distance': LaunchConfiguration('safety_distance'),
                 'reached_dist_thresh': LaunchConfiguration('reached_dist_thresh'),
                 'spin_yaw_thresh': LaunchConfiguration('spin_yaw_thresh'),
                 'navigation_speed': LaunchConfiguration('navigation_speed'),
                 'cautiously': LaunchConfiguration('cautiously'),
                 'spin_speed': LaunchConfiguration('spin_speed')},
                LaunchConfiguration('config_file')
            ],
            output="screen",
            emulate_tty=True,
            # arguments=['--ros-args', '--log-level', 'DEBUG']
        ),
    ])
