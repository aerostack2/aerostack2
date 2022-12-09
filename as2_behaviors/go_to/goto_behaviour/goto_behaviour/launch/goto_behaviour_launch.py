from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('goto_behaviour'),
        'config', 'goto_behaviour.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=EnvironmentVariable(
            'AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgument('config', default_value=config),
        Node(
            package='goto_behaviour',
            executable='goto_behaviour_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[LaunchConfiguration('config')],
            output='screen',
            emulate_tty=True
        )
    ])
