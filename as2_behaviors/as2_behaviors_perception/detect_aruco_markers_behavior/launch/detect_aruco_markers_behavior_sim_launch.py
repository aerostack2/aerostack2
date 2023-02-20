""" Launch file for aruco detector node. """

import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """ Launch aruco detector node. """

    config = os.path.join(get_package_share_directory('as2_behaviors_perception'),
                          'detect_aruco_markers_behavior/config/sim_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=EnvironmentVariable(
            'AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument(
            'camera_image_topic', default_value='sensor_measurements/camera/image_raw'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='sensor_measurements/camera/camera_info'),
        Node(
            package='as2_behaviors_perception',
            executable='detect_aruco_markers_behavior_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            arguments=['--ros-args', '--log-level',
                       LaunchConfiguration('log_level')],
            parameters=[{'camera_image_topic': LaunchConfiguration('camera_image_topic'),
                         'camera_info_topic': LaunchConfiguration('camera_info_topic')},
                        config],
            emulate_tty=True,
        ),
    ])
