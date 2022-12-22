from os.path import join

import launch
from launch import LaunchDescription, conditions
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = join(
        get_package_share_directory('as2_crazyflie_platform'),
        'config',
        'control_modes.yaml'
    )
    swarm_config = join(
        get_package_share_directory('as2_crazyflie_platform'),
        'config',
        'crazy_swarm.yaml'
    )
    # DRONE_ID = os.environ['AEROSTACK2_SIMULATION_DRONE_ID']
    return LaunchDescription([
        # DeclareLaunchArgument('drone_id', default_value=DRONE_ID, description='Drone namespace.'),
        DeclareLaunchArgument('mass', default_value='0.029'),
        DeclareLaunchArgument('max_thrust', default_value='0.0'),
        DeclareLaunchArgument('min_thrust', default_value='0.0'),
        DeclareLaunchArgument('control_modes_file', default_value=config),
        DeclareLaunchArgument('swarm_config_file', default_value=swarm_config),
        DeclareLaunchArgument('drone_URI', default_value='radio://0/80/2M/E7E7E7E7E7', description='Crazyflie URI.'),
        DeclareLaunchArgument('external_odom', default_value='false', choices=["true", "false"], description='Availability of external odometry.'),
        DeclareLaunchArgument('external_odom_topic', default_value='external_odom', description='External odometry topic name.'),
        DeclareLaunchArgument('simulation_mode', default_value='false', choices=["true", "false"], description="Simulation flag."),
        DeclareLaunchArgument('controller_type', default_value='1', choices=["0", "1", "2", "3"], description="Controller type Any(0), PID(1), Mellinger(2), INDI(3) (Default: 0)."),
        DeclareLaunchArgument('estimator_type', default_value='1', choices=["0", "1", "2"], description="Estimator type Any(0), complementary(1), kalman(2) (Default: 0)."),
        DeclareLaunchArgument('ip', default_value="192.168.43.95"),
        DeclareLaunchArgument('port', default_value='5000'),
        DeclareLaunchArgument('save_flag', default_value='False'),
        DeclareLaunchArgument('show_flag', default_value='False'),
        # if is not in simulation
        Node(
            package="as2_crazyflie_platform",
            executable="as2_crazyflie_platform_swarm_node",
            name="platform",
            # namespace=LaunchConfiguration('drone_id'),
            output="screen",
            emulate_tty=True,
            parameters=[
                {"mass": LaunchConfiguration('mass'),
                "max_thrust": LaunchConfiguration('max_thrust'),
                "control_modes_file": LaunchConfiguration('control_modes_file'),
                "swarm_config_file": LaunchConfiguration('swarm_config_file'),
                "external_odom" : LaunchConfiguration('external_odom'),
                "drone_URI" : LaunchConfiguration('drone_URI'),
                "external_odom_topic" : LaunchConfiguration('external_odom_topic'),
                "min_thrust": LaunchConfiguration('min_thrust'),
                "simulation_mode": LaunchConfiguration('simulation_mode'),
                "controller_type": LaunchConfiguration('controller_type'),
                "estimator_type": LaunchConfiguration('estimator_type'),
                }],
            #remappings=[("sensor_measurements/odometry", "self_localization/odom")],
        )
    ])
