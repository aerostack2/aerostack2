"""Launch Pixhawk platform node"""
from os.path import join
import yaml
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch import LaunchDescription


def get_platform_node(context):
    """Build pixhawk platform node"""
    config = LaunchConfiguration('config').perform(context)

    with open(config, "r", encoding='utf-8') as file_:
        config_params = yaml.safe_load(file_)

    try:
        control_modes_file = config_params["/**"]["ros__parameters"]["control_modes_file"]
    except KeyError:
        control_modes_file = ""

    if not control_modes_file:
        control_modes_file = join(
            get_package_share_directory('as2_pixhawk_platform'),
            'config', 'control_modes.yaml'
        )

    dict_ = {'/**': {'ros__parameters': {'control_modes_file': f'{control_modes_file}'}
                     }}
    with open('/tmp/aux_config.yaml', 'w', encoding='utf-8') as file_:
        yaml.dump(dict_, file_, default_flow_style=False)

    node = Node(
        package="as2_pixhawk_platform",
        executable="as2_pixhawk_platform_node",
        name="platform",
        namespace=LaunchConfiguration('namespace'),
        output="screen",
        emulate_tty=True,
        parameters=[config,
                    '/tmp/aux_config.yaml',
                    {'use_sim_time': LaunchConfiguration('use_sim_time'),
                     'external_odom': LaunchConfiguration('external_odom')}],
    )

    return [node]


def generate_launch_description():
    """Entrypoint"""
    config = PathJoinSubstitution([
        FindPackageShare('as2_pixhawk_platform'),
        'config', 'platform_default.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=EnvironmentVariable(
            'AEROSTACK2_SIMULATION_DRONE_ID'), description='Drone namespace'),
        DeclareLaunchArgument('config', default_value=config,
                              description='Configuration file'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=[
                              'true', 'false'], description='Use simulation time'),
        DeclareLaunchArgument('external_odom', default_value='true', choices=[
                              'true', 'false'], description='Use external odometry'),
        OpaqueFunction(function=get_platform_node)
    ])
