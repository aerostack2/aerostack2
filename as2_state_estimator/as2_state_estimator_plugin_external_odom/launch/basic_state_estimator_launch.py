from launch import LaunchDescription

from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='drone0'),
        DeclareLaunchArgument('use_sim_time', default_value='False'),
        DeclareLaunchArgument('odom_only', default_value='False'),
        DeclareLaunchArgument('ground_truth', default_value='False'),
        DeclareLaunchArgument('sensor_fusion', default_value='False'),
        DeclareLaunchArgument('rectified_localization', default_value='False'),
        DeclareLaunchArgument('global_ref_frame', default_value='/earth'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        Node(
            package='basic_state_estimator',
            executable='basic_state_estimator_node',
            name='basic_state_estimator',
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                {'odom_only': LaunchConfiguration('odom_only')},
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'ground_truth': LaunchConfiguration('ground_truth')},
                {'sensor_fusion': LaunchConfiguration('sensor_fusion')},
                {'rectified_localization': LaunchConfiguration('rectified_localization')},
                {'global_ref_frame': LaunchConfiguration('global_ref_frame')},
                {'base_frame': LaunchConfiguration('base_frame')}],
            output='screen',
            emulate_tty=True,
            # remappings=[("ground_truth/pose","global_localization/pose")]
            # remappings=[("rectified_localization/pose","global_localization/pose")]
        )
    ])
