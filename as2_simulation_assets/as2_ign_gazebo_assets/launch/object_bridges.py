from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo, Shutdown
from launch.substitutions import LaunchConfiguration

from ign_assets.model import ObjectModel
import json


def object_bridges(context, *args, **kwargs):
    config_file = LaunchConfiguration('config_file').perform(context)

    with open(config_file, 'r') as stream:
        config = json.load(stream)
        if 'world' not in config:
            raise RuntimeError(
                'Cannot construct bridges without world in config')
        world_name = config['world']

    with open(config_file, 'r') as stream:
        # return only objects tagged models
        object_models = ObjectModel.FromConfig(stream)

    nodes = []
    for object_model in object_models:
        bridges, custom_bridges = object_model.bridges(world_name)
        nodes.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            namespace=object_model.model_name,
            output='screen',
            arguments=[bridge.argument() for bridge in bridges],
            remappings=[bridge.remapping() for bridge in bridges]
        ))
        nodes += custom_bridges

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            description='YAML configuration file to spawn'
        ),
        OpaqueFunction(function=object_bridges)
    ])
