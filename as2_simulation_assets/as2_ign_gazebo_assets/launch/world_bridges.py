from launch import LaunchDescription
from launch_ros.actions import Node

import ign_assets.bridges


def generate_launch_description():
    bridges = [
        ign_assets.bridges.clock()
    ]
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            namespace='world',
            output='screen',
            arguments=[bridge.argument() for bridge in bridges],
            remappings=[bridge.remapping() for bridge in bridges]
        )
    ])
