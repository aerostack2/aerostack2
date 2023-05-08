"""Launch Crazyflie Swarm platform node"""
import logging
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

FORMAT = '[%(levelname)s] [launch]: %(message)s'
logging.basicConfig(format=FORMAT)


def get_platform_node(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)

    cmd_vel_topic = DeclareLaunchArgument(
        'cmd_vel_topic', default_value=f'/ign/{namespace}/cmd_vel')
    arm_topic = DeclareLaunchArgument(
        'arm_topic', default_value=f'/ign/{namespace}/arm')

    node = Node(
        package="as2_platform_ign_gazebo",
        executable="as2_platform_ign_gazebo_node",
        namespace=LaunchConfiguration('namespace'),
        output="screen",
        emulate_tty=True,
        parameters=[{
            "use_sim_time": LaunchConfiguration('use_sim_time'),
            "control_modes_file": LaunchConfiguration('control_modes_file'),
            "cmd_vel_topic": LaunchConfiguration('cmd_vel_topic'),
            "arm_topic": LaunchConfiguration('arm_topic')
        },
            LaunchConfiguration('platform_config_file')
        ]
    )
    return [cmd_vel_topic, arm_topic, node]


def generate_launch_description():
    """Entrypoint"""

    control_modes = PathJoinSubstitution([
        FindPackageShare('as2_platform_ign_gazebo'),
        'config', 'control_modes.yaml'
    ])

    platform_config_file = PathJoinSubstitution([
        FindPackageShare('as2_platform_ign_gazebo'),
        'config', 'platform_config_file.yaml'
    ])

    drone_bridges = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([
            FindPackageShare('as2_ign_gazebo_assets'),
            'launch', 'drone_bridges.py'
        ])]),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'simulation_config_file': LaunchConfiguration('simulation_config_file')
        }.items(),
    )

    logging.critical(platform_config_file)

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=EnvironmentVariable(
            'AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('control_modes_file',
                              default_value=control_modes,
                              description='Platform control modes file'),
        DeclareLaunchArgument('platform_config_file',
                              default_value=platform_config_file,
                              description='Platform configuration file'),
        DeclareLaunchArgument(
            'simulation_config_file', description='JSON configuration file to create bridges'),
        drone_bridges,
        OpaqueFunction(function=get_platform_node)
    ])
