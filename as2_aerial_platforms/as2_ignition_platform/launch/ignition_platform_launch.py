from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def get_platform_node(context, *args, **kwargs):
    drone_id = LaunchConfiguration('drone_id').perform(context)

    cmd_vel_topic = DeclareLaunchArgument(
        'cmd_vel_topic', default_value=f'/ign/{drone_id}/cmd_vel')
    arm_topic = DeclareLaunchArgument(
        'arm_topic', default_value=f'/ign/{drone_id}/arm')
    node = Node(
        package="as2_ignition_platform",
        executable="as2_ignition_platform_node",
        namespace=LaunchConfiguration('drone_id'),
        output="screen",
        emulate_tty=True,
        parameters=[{
            "use_sim_time": LaunchConfiguration('use_sim_time'),
            "control_modes_file": LaunchConfiguration('control_modes_file'),
            "cmd_vel_topic": LaunchConfiguration('cmd_vel_topic'),
            "arm_topic": LaunchConfiguration('arm_topic'),
            "enable_takeoff_platform": LaunchConfiguration('enable_takeoff_platform'),
            "enable_land_platform": LaunchConfiguration('enable_land_platform')
        }]
    )
    return [cmd_vel_topic, arm_topic, node]


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('as2_ignition_platform'),
        'config', 'control_modes.yaml'
    ])

    bridges = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([
            FindPackageShare('ignition_assets'),
            'launch', 'model_bridges.py'
        ])]),
        launch_arguments={
            'drone_id': LaunchConfiguration('drone_id'),
            'config_file': LaunchConfiguration('config_file')
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value=EnvironmentVariable(
            'AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('control_modes_file', default_value=config),
        DeclareLaunchArgument(
            'config_file', description='JSON configuration file to create bridges'),
        DeclareLaunchArgument('enable_takeoff_platform', default_value='false',
                              description='Enable takeoff platform, only for debugging purposes'),
        DeclareLaunchArgument('enable_land_platform', default_value='false',
                              description='Enable land platform, only for debugging purposes'),
        bridges,
        OpaqueFunction(function=get_platform_node)
    ])
