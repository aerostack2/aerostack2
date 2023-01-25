from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import UnlessCondition, IfCondition

import yaml
from os.path import join
from ament_index_python.packages import get_package_share_directory


def get_platform_node(context, *args, **kargs):
    config = LaunchConfiguration('config').perform(context)
    simulation_mode = bool(LaunchConfiguration('simulation_mode').perform(context))
    with open(config, "r") as f:
        config_params = yaml.safe_load(f)

    try:
        control_modes_file = config_params["/**"]["ros__parameters"]["control_modes_file"]
    except KeyError:
        control_modes_file = ""

    if not control_modes_file:
        control_modes_file = join(
            get_package_share_directory('as2_platform_dji_osdk'),
            'config', 'control_modes.yaml'
        )

    dict = {'/**': {'ros__parameters': {'simulation_mode': simulation_mode, 
                                        'control_modes_file': f'{control_modes_file}'}
            }}
    with open('/tmp/aux_config.yaml', 'w') as f:
        yaml.dump(dict, f, default_flow_style=False)

    node = Node(
        package="as2_platform_dji_osdk",
        executable="as2_platform_dji_osdk_node",
        name="platform",
        namespace=LaunchConfiguration('namespace'),
        output="screen",
        emulate_tty=True,
        parameters=[config, '/tmp/aux_config.yaml'],
        arguments=[LaunchConfiguration('dji_app_config')],
        remappings=[("sensor_measurements/odometry", "self_localization/odom")]
    )

    return [node]


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('as2_platform_dji_osdk'),
        'config', 'platform_default.yaml'
    ])
    dji_app_config = PathJoinSubstitution([
        FindPackageShare('as2_platform_dji_osdk'),
        'config', 'UserConfig.txt'
    ])
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=EnvironmentVariable('AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument('dji_app_config', default_value=dji_app_config),
        DeclareLaunchArgument('simulation_mode', default_value='false'),
        OpaqueFunction(function=get_platform_node)
    ])
