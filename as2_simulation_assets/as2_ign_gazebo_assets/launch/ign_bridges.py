from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

import ign_assets.bridges
from ign_assets.model import Model
import json

#
# NOT INTENDED TO USE!! USE MODEL AND WORLD BRIDGES INSTEAD.
# THIS LAUNCH FILE CREATES BRIDGES FOR ALL THE CONFIG FILE, INCLUDED WORLD AND ALL MODELS INSIDE.
# BRIDGES SHOULD BE CREATED WITHIN THE PLATFORM LAUNCH.
# 


def model_bridges(context, *args, **kwargs):
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

    print(nodes)
    return nodes


def general_bridges(context, *args, **kwargs):
    bridges = [
        ign_assets.bridges.clock()
    ]

    nodes = []
    nodes.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[bridge.argument() for bridge in bridges],
        remappings=[bridge.remapping() for bridge in bridges]
    ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            description='YAML configuration file to spawn'
        ),
        OpaqueFunction(function=general_bridges),
        OpaqueFunction(function=model_bridges)
    ])
