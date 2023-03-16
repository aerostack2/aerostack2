from os.path import join

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = join(
        get_package_share_directory('as2_platform_crazyflie'),
        'config',
        'control_modes.yaml'
    )

    # DRONE_ID = os.environ['AEROSTACK2_SIMULATION_DRONE_ID']
    return LaunchDescription([
        DeclareLaunchArgument('control_modes_file', default_value=config, description='Available control modes file.'),
        DeclareLaunchArgument('swarm_config_file', description='Swarm URIs configuration file.'),
        DeclareLaunchArgument('external_odom', default_value='false', choices=["true", "false"], description='Availability of external odometry.'),
        DeclareLaunchArgument('external_odom_topic', default_value='external_odom', description='External odometry topic name.'),
        DeclareLaunchArgument('controller_type', default_value='1', choices=["0", "1", "2", "3"], description="Controller type Any(0), PID(1), Mellinger(2), INDI(3) (Default: 0)."),
        DeclareLaunchArgument('estimator_type', default_value='2', choices=["0", "1", "2"], description="Estimator type Any(0), complementary(1), kalman(2) (Default: 0)."),
        Node(
            package="as2_platform_crazyflie",
            executable="as2_platform_crazyflie_swarm_node",
            name="platform",
            # namespace=LaunchConfiguration('drone_id'),
            output="screen",
            emulate_tty=True,
            parameters=[{
                "control_modes_file": LaunchConfiguration('control_modes_file'),
                "swarm_config_file": LaunchConfiguration('swarm_config_file'),
                "external_odom" : LaunchConfiguration('external_odom'),
                "external_odom_topic" : LaunchConfiguration('external_odom_topic'),
                "controller_type": LaunchConfiguration('controller_type'),
                "estimator_type": LaunchConfiguration('estimator_type'),
                }],
        )
    ])
