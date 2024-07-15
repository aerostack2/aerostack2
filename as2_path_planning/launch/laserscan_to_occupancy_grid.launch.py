"""
laserscan_to_occupancy_grid.py
"""
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable


def generate_launch_description():
    """entrypoint
    """
    return LaunchDescription([
        DeclareLaunchArgument('namespace', description="Drone namespace",
                              default_value=EnvironmentVariable('AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgument(
            'use_sim_time', description="Use sim time flag", default_value='false'),
        DeclareLaunchArgument(
            'map_resolution', description="Map spatial resolution [m/cell]", default_value='0.25'),
        DeclareLaunchArgument(
            'map_width', description="Map width [cell]", default_value='300'),
        DeclareLaunchArgument(
            'map_height', description="Map height [cell]", default_value='300'),
        DeclareLaunchArgument(
            'max_range_limit', description="Maximum sensor range limit [m]", default_value='10.0'),
        DeclareLaunchArgument(
            'output_topic',
            description="Output topic where the occupancy grid is published",
            default_value='output_occupancy_grid'),
        DeclareLaunchArgument(
            'config_file', description="Path to config file. " +
            "Be careful, parameters in file will override launch arguments",
            default_value=''),
        Node(
            package="as2_path_planning",
            executable="as2_path_planning_node",
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time'),
                 'map_resolution': LaunchConfiguration('map_resolution'),
                 'map_width': LaunchConfiguration('map_width'),
                 'map_height': LaunchConfiguration('map_height'),
                 'max_range_limit': LaunchConfiguration('max_range_limit')},
                LaunchConfiguration('config_file')
            ],
            remappings=[("labeled_occ_grid",
                         LaunchConfiguration('output_topic'))],
            output="screen",
            emulate_tty=True,
            # arguments=['--ros-args', '--log-level', 'DEBUG']
        ),
    ])
