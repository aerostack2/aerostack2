from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo, Shutdown
from launch.substitutions import LaunchConfiguration

from ign_assets.model import DroneModel
import json


def drone_bridges(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    config_file = LaunchConfiguration('config_file').perform(context)

    with open(config_file, 'r') as stream:
        config = json.load(stream)
        if 'world' not in config:
            raise RuntimeError(
                'Cannot construct bridges without world in config')
        world_name = config['world']

    with open(config_file, 'r') as stream:
        # return only drones tagged models
        drone_models = DroneModel.FromConfig(stream)

    nodes = []
    for drone_model in drone_models:
        if drone_model.model_name == namespace:
            bridges, custom_bridges = drone_model.bridges(world_name)
            nodes.append(Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                namespace=drone_model.model_name,
                output='screen',
                arguments=[bridge.argument() for bridge in bridges],
                remappings=[bridge.remapping() for bridge in bridges]
            ))
            nodes += custom_bridges

    if not nodes:
        return [
            LogInfo(msg="Gazebo Ignition bridge creation failed."),
            LogInfo(msg=f"Drone ID: {namespace} not found in {config_file}."),
            Shutdown(reason=f"Aborting..")]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            description='YAML configuration file to spawn'
        ),
        DeclareLaunchArgument(
            'namespace',
            description='Drone ID to create bridges'
        ),
        OpaqueFunction(function=drone_bridges)
    ])
