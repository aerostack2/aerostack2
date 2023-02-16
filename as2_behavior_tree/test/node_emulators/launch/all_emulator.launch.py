"""All emulator launcher
"""
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable


def generate_launch_description():
    """Launch description
    """
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', description="Drone namespace",
                              default_value=EnvironmentVariable('AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgument(
            'use_sim_time', description="Use sim time flag", default_value='false'),

        Node(
            package="node_emulators",
            executable="platform_emulator",
            namespace=LaunchConfiguration('drone_id'),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output="screen",
            emulate_tty=True,
            # arguments=['--ros-args', '--log-level', 'DEBUG']
        ),
        Node(
            package="node_emulators",
            executable="takeoff_emulator",
            namespace=LaunchConfiguration('drone_id'),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output="screen",
            emulate_tty=True,
            # arguments=['--ros-args', '--log-level', 'DEBUG']
        ),
        Node(
            package="node_emulators",
            executable="land_emulator",
            namespace=LaunchConfiguration('drone_id'),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output="screen",
            emulate_tty=True,
            # arguments=['--ros-args', '--log-level', 'DEBUG']
        ),
        Node(
            package="node_emulators",
            executable="go_to_emulator",
            namespace=LaunchConfiguration('drone_id'),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output="screen",
            emulate_tty=True,
            # arguments=['--ros-args', '--log-level', 'DEBUG']
        ),
        Node(
            package="node_emulators",
            executable="follow_path_emulator",
            namespace=LaunchConfiguration('drone_id'),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output="screen",
            emulate_tty=True,
            # arguments=['--ros-args', '--log-level', 'DEBUG']
        ),
    ])
