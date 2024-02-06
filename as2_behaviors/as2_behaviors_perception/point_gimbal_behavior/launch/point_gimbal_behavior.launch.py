""" Launch file for point gimbal behavior node. """

import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """ Launch point gimbal behavior node. """

    # config = os.path.join(get_package_share_directory('as2_behaviors_perception'),
    #                       'point_gimbal_behavior/config/params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=EnvironmentVariable(
            'AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        Node(
            package='as2_behaviors_perception',
            executable='point_gimbal_behavior_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            arguments=['--ros-args', '--log-level',
                       LaunchConfiguration('log_level')],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            emulate_tty=True,
        ),
    ])
