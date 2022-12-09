from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    config_takeoff = PathJoinSubstitution([
        FindPackageShare('takeoff_behaviour'),
        'config', 'takeoff_behaviour.yaml'
    ])
    config_land = PathJoinSubstitution([
        FindPackageShare('land_behaviour'),
        'config', 'land_behaviour.yaml'
    ])
    config_goto = PathJoinSubstitution([
        FindPackageShare('goto_behaviour'),
        'config', 'goto_behaviour.yaml'
    ])
    config_follow_path = PathJoinSubstitution([
        FindPackageShare('follow_path_behaviour'),
        'config', 'follow_path_behaviour.yaml'
    ])

    takeoff = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([
            FindPackageShare('takeoff_behaviour'),
            'launch', 'takeoff_behaviour_launch.py'
        ])]),
        launch_arguments={
            'drone_id': LaunchConfiguration('drone_id'),
            'config': LaunchConfiguration('config_takeoff')
        }.items(),
    )

    land = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([
            FindPackageShare('land_behaviour'),
            'launch', 'land_behaviour_launch.py'
        ])]),
        launch_arguments={
            'drone_id': LaunchConfiguration('drone_id'),
            'config': LaunchConfiguration('config_land')
        }.items(),
    )

    goto = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([
            FindPackageShare('goto_behaviour'),
            'launch', 'goto_behaviour_launch.py'
        ])]),
        launch_arguments={
            'drone_id': LaunchConfiguration('drone_id'),
            'config': LaunchConfiguration('config_goto')
        }.items(),
    )

    follow_path = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([
            FindPackageShare('follow_path_behaviour'),
            'launch', 'follow_path_behaviour_launch.py'
        ])]),
        launch_arguments={
            'drone_id': LaunchConfiguration('drone_id'),
            'config': LaunchConfiguration('config_follow_path')
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value=EnvironmentVariable('AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgument('config_takeoff', default_value=config_takeoff),
        DeclareLaunchArgument('config_land', default_value=config_land),
        DeclareLaunchArgument('config_goto', default_value=config_goto),
        DeclareLaunchArgument('config_follow_path', default_value=config_follow_path),
        takeoff,
        land,
        goto,
        follow_path
    ])
