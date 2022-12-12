from launch import LaunchDescription

from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='drone0'),
        DeclareLaunchArgument('use_sim_time', default_value='False'),
        DeclareLaunchArgument('global_ref_frame', default_value='/earth'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        Node(
            package='basic_state_estimator',
            executable='mocap_state_estimator_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'global_ref_frame': LaunchConfiguration('global_ref_frame')},
                {'base_frame': LaunchConfiguration('base_frame')}],
            output='screen',
            emulate_tty=True,
        )
    ])
