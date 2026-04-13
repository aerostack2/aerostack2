from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare('as2_camera_overlay'),
        'config',
        'synthetic.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='Path to parameters YAML file.'),
        DeclareLaunchArgument(
            'name',
            default_value='as2_camera_overlay',
            description='Node name.'),
        Node(
            package='as2_camera_overlay',
            executable='as2_camera_overlay_node',
            name=LaunchConfiguration('name'),
            output='screen',
            parameters=[LaunchConfiguration('config')],
        ),
    ])
