from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('follow_path_behaviour'),
        'config', 'follow_path_behaviour.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=EnvironmentVariable('AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='follow_path_behaviour',
            executable='follow_path_behaviour_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[LaunchConfiguration('config'),
                        {"use_sim_time": LaunchConfiguration('use_sim_time')}],
            output='screen',
            emulate_tty=True
        )
    ])