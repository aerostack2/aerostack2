"""
platform_gazebo_launch.py
"""
import logging
from launch_ros.actions import LoadComposableNodes, Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

FORMAT = '[%(levelname)s] [launch]: %(message)s'
logging.basicConfig(format=FORMAT)


def get_platform_node(context, *args, **kwargs):
    """Get platform node"""
    namespace = LaunchConfiguration('namespace').perform(context)

    cmd_vel_topic = DeclareLaunchArgument(
        'cmd_vel_topic', default_value=f'/gz/{namespace}/cmd_vel')
    arm_topic = DeclareLaunchArgument(
        'arm_topic', default_value=f'/gz/{namespace}/arm')

    node = Node(
        package="as2_platform_gazebo",
        executable="as2_platform_gazebo_node",
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
        FindPackageShare('as2_platform_gazebo'),
        'config', 'control_modes.yaml'
    ])

    platform_config_file = PathJoinSubstitution([
        FindPackageShare('as2_platform_gazebo'),
        'config', 'platform_config_file.yaml'
    ])

    # drone_bridges = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([PathJoinSubstitution([
    #         FindPackageShare('as2_gazebo_assets'),
    #         'launch', 'drone_bridges.py'
    #     ])]),
    #     launch_arguments={
    #         'namespace': 'drone0',
    #         'simulation_config_file': LaunchConfiguration('simulation_config_file')
    #     }.items(),
    # )

    logging.critical(platform_config_file)

    container = ComposableNodeContainer(
        name='container',   # Nombre container donde se indexa
        namespace='drone0',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='as2_platform_gazebo',
                plugin='gazebo_platform::GazeboPlatform',
                name='GazeboPlatform',
                namespace='drone0',
                parameters=[{
                    'use_sim_time': True,
                    'control_modes_file': control_modes,
                    'platform_config_file': platform_config_file,
                    'enable_takeoff_platform': True,
                    'enable_land_platform': True,
                    'simulation_config_file':'simulation_config',
                    'cmd_vel_topic':'gz/drone0/cmd_vel',
                    'arm_topic':'gz/drone0/arm'
                }])
            ],
            output='both',
        )

    # container = ComposableNodeContainer(
    #     name='container',
    #     namespace='drone0',
    #     package='rclcpp_components',
    #     executable='component_container',
    #     composable_node_descriptions=[cnode],
    #     output='screen',
    # )

    launch_description = LaunchDescription([container])

    return launch_description#, drone_bridges
