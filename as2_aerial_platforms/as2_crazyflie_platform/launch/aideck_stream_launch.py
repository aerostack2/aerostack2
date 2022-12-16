from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ip', default_value="192.168.43.95"),
        DeclareLaunchArgument('port', default_value='5000'),
        DeclareLaunchArgument('save_flag', default_value='False'),
        DeclareLaunchArgument('show_flag', default_value='False'),
        Node(
            package="as2_crazyflie_platform",
            executable='aideck_node.py',
            name='aideck_pub',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'ip': LaunchConfiguration('ip'),
                'port': LaunchConfiguration('port'),
                'save_flag': LaunchConfiguration('save_flag'),
                'show_flag': LaunchConfiguration('show_flag'),
                }
            ]
        )
    ])
