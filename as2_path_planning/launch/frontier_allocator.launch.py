"""
fontier_allocator.launch.py
"""
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch frontier allocator"""
    return LaunchDescription([
        DeclareLaunchArgument('namespace', description="Namespace",
                              default_value=''),
        DeclareLaunchArgument(
            'use_sim_time', description="Use sim time flag", default_value='false'),
        DeclareLaunchArgument(
            'safety_distance', description="Drone safety distance on obstacles [m]",
            default_value='0.25'),
        DeclareLaunchArgument(
            'frontier_min_area', description="Frontier minimum area [cell]", default_value='15'),
        DeclareLaunchArgument(
            'frontier_max_area', description="Frontier maximum area [cell]", default_value='25'),
        DeclareLaunchArgument(
            'config_file', description="Path to config file. " +
            "Be careful, parameters in file will override launch arguments",
            default_value=''),
        Node(
            package="as2_path_planning",
            executable="frontier_allocator",
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time'),
                 'safety_distance': LaunchConfiguration('safety_distance'),
                 'frontier_min_area': LaunchConfiguration('frontier_min_area'),
                 'frontier_max_area': LaunchConfiguration('frontier_max_area')},
                LaunchConfiguration('config_file')
            ],
            output="screen",
            emulate_tty=True,
            # arguments=['--ros-args', '--log-level', 'DEBUG']
        ),
    ])
