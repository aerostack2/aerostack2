from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo, Shutdown
from launch.substitutions import LaunchConfiguration

import ign_assets.bridges

def world_bridges(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time = use_sim_time.lower() in ['true', 't', 'yes', 'y', '1']
    bridges = [
    ]
    if use_sim_time:
        bridges.append(ign_assets.bridges.clock())
    nodes = []
    node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            namespace='world',
            output='screen',
            arguments=[bridge.argument() for bridge in bridges],
            remappings=[bridge.remapping() for bridge in bridges]
        )
    nodes.append(node)

    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            description='Make objects publish tfs in sys clock time or sim time'
        ),
        OpaqueFunction(function=world_bridges)
    ])
