""" Launch file for the motion behavior """

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


BEHAVIOR_NAME = 'follow_reference'


def get_node(context):
    """ Returns the behavior node """
    behavior_config_file = LaunchConfiguration(
        'behavior_config_file').perform(context)

    parameters = [{"use_sim_time": LaunchConfiguration('use_sim_time')}]

    if not behavior_config_file:
        behavior_config_file = PathJoinSubstitution([
            FindPackageShare('as2_behaviors_motion'),
            'config/' + BEHAVIOR_NAME + '_behavior/config_default.yaml'
        ])

    parameters.append(behavior_config_file)

    node = Node(
        package='as2_behaviors_motion',
        executable=BEHAVIOR_NAME + '_behavior_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=parameters,
        output='screen',
        emulate_tty=True
    )

    return [node]


def generate_launch_description():
    """ Returns the launch description """
    launch_description = LaunchDescription([
        DeclareLaunchArgument('namespace'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('behavior_config_file', default_value='',
                              description='Path to behavior config file'),
        OpaqueFunction(function=get_node)
    ])

    return launch_description
