""" Launch file for the follow_reference behavior """
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """ Launch follow reference behavior. """
    return LaunchDescription([
        DeclareLaunchArgument('namespace'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('follow_reference_max_speed_x', default_value="10.0"),
        DeclareLaunchArgument('follow_reference_max_speed_y', default_value="10.0"),
        DeclareLaunchArgument('follow_reference_max_speed_z', default_value="1.0"),
        Node(
            package='as2_behaviors_motion',
            executable='follow_reference_behavior_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                {"namespace": LaunchConfiguration('namespace')},
                {"use_sim_time": LaunchConfiguration('use_sim_time')},
                {"follow_reference_max_speed_x": LaunchConfiguration('follow_reference_max_speed_x')},
                {"follow_reference_max_speed_y": LaunchConfiguration('follow_reference_max_speed_y')},
                {"follow_reference_max_speed_z": LaunchConfiguration('follow_reference_max_speed_z')}           
            ],
            output='screen',
            emulate_tty=True
        ),
    ])