from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('land_behaviour'),
        'config', 'land_behaviour.yaml'
    ])


    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value=EnvironmentVariable('AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgument('config', default_value=config),
        Node(
            package='land_behaviour',
            executable='land_behaviour_node',
            namespace=LaunchConfiguration('drone_id'),
            parameters=[LaunchConfiguration('config')],
            output='screen',
            emulate_tty=True
        )
    ])