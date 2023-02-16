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
    config_go_to = PathJoinSubstitution([
        FindPackageShare('go_to_behaviour'),
        'config', 'go_to_behaviour.yaml'
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

    go_to = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([
            FindPackageShare('go_to_behaviour'),
            'launch', 'go_to_behaviour_launch.py'
        ])]),
        launch_arguments={
            'drone_id': LaunchConfiguration('drone_id'),
            'config': LaunchConfiguration('config_go_to')
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
        DeclareLaunchArgument('drone_id', default_value=EnvironmentVariable(
            'AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgument('config_takeoff', default_value=config_takeoff),
        DeclareLaunchArgument('config_land', default_value=config_land),
        DeclareLaunchArgument('config_go_to', default_value=config_go_to),
        DeclareLaunchArgument('config_follow_path',
                              default_value=config_follow_path),
        takeoff,
        land,
        go_to,
        follow_path
    ])
