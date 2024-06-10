"""
platform_gazebo_launch.py
"""
from launch_ros.actions import LoadComposableNodes, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals


def get_platform_node(context):
    """Get platform node"""
    namespace = LaunchConfiguration('namespace').perform(context)

    cmd_vel_topic = DeclareLaunchArgument(
        'cmd_vel_topic', default_value=f'/gz/{namespace}/cmd_vel')
    arm_topic = DeclareLaunchArgument(
        'arm_topic', default_value=f'/gz/{namespace}/arm')

    return [cmd_vel_topic, arm_topic]


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

    drone_bridges = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([
            FindPackageShare('as2_gazebo_assets'),
            'launch', 'drone_bridges.py'
        ])]),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'simulation_config_file': LaunchConfiguration('simulation_config_file')
        }.items(),
    )

    node = ComposableNode(
        package='as2_platform_gazebo',
        plugin='gazebo_platform::GazeboPlatform',
        # name='GazeboPlatform',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            "use_sim_time": LaunchConfiguration('use_sim_time'),
            "control_modes_file": LaunchConfiguration('control_modes_file'),
            "cmd_vel_topic": LaunchConfiguration('cmd_vel_topic'),
            "arm_topic": LaunchConfiguration('arm_topic')
        },
            LaunchConfiguration('platform_config_file')
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    container = ComposableNodeContainer(
        condition=LaunchConfigurationEquals('container', ''),
        name=LaunchConfiguration('container'),
        namespace='drone0',
        package='rclcpp_components',
        executable='component_container_isolated',
        composable_node_descriptions=[
                node
            ],
            output='both',
        )

    load_on_existing_container = LoadComposableNodes(
        condition=LaunchConfigurationNotEquals('container', ''),
        composable_node_descriptions=[
            node
        ],
        target_container=(LaunchConfiguration('namespace'), '/aerostack2'),
    )

    launch_description = LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=EnvironmentVariable(
            'AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgument('container', default_value='aerostack2'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('control_modes_file',
                              default_value=control_modes,
                              description='Platform control modes file'),
        DeclareLaunchArgument('platform_config_file',
                              default_value=platform_config_file,
                              description='Platform configuration file'),
        DeclareLaunchArgument(
            'simulation_config_file', description='JSON configuration file to create bridges'),
        OpaqueFunction(function=get_platform_node),
        drone_bridges,
        container,
        load_on_existing_container
    ])

    return launch_description
