from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=EnvironmentVariable(
            'AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        Node(
            package='as2_behaviors_trajectory_generation',
            executable='generate_polynomial_trajectory_behavior_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            arguments=['--ros-args', '--log-level',
                       LaunchConfiguration('log_level')],
            parameters=[{"use_sim_time": LaunchConfiguration('use_sim_time')}],
            emulate_tty=True
        )
    ])
