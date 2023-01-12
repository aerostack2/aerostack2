from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo, Shutdown
from launch.substitutions import LaunchConfiguration

from ign_assets.model import Model
import json


def model_bridges(context, *args, **kwargs):
    drone_id = LaunchConfiguration('drone_id').perform(context)
    config_file = LaunchConfiguration('config_file').perform(context)

    with open(config_file, 'r') as stream:
        config = json.load(stream)
        if 'world' not in config:
            raise RuntimeError('Cannot construct bridges without world in config')
        world_name = config['world']

    with open(config_file, 'r') as stream:
        models = Model.FromConfig(stream)

    nodes = []
    for model in models:
        if model.model_name == drone_id:
            bridges, custom_bridges = model.bridges(world_name)
            nodes.append(Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                namespace=model.model_name,
                output='screen',
                arguments=[bridge.argument() for bridge in bridges],
                remappings=[bridge.remapping() for bridge in bridges]
            ))
            nodes += custom_bridges

    if not nodes:
        return [
            LogInfo(msg="Gazebo Ignition bridge creation failed."),
            LogInfo(msg=f"Drone ID: {drone_id} not found in {config_file}."),
            Shutdown(reason=f"Aborting..")]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            description='YAML configuration file to spawn'
        ),
        DeclareLaunchArgument(
            'drone_id',
            description='Drone ID to create bridges'
        ),
        OpaqueFunction(function=model_bridges)
    ])
