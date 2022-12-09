from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


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
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value=EnvironmentVariable('AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('config_takeoff', default_value=config_takeoff),
        DeclareLaunchArgument('config_land', default_value=config_land),
        DeclareLaunchArgument('config_goto', default_value=config_goto),
        DeclareLaunchArgument('config_follow_path', default_value=config_follow_path),
        Node(
            package='takeoff_behaviour',
            executable='takeoff_behaviour_node',
            namespace=LaunchConfiguration('drone_id'),
            parameters=[
                {"use_sim_time": LaunchConfiguration('use_sim_time')},
                LaunchConfiguration('config_takeoff')],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='land_behaviour',
            executable='land_behaviour_node',
            namespace=LaunchConfiguration('drone_id'),
            parameters=[
                {"use_sim_time": LaunchConfiguration('use_sim_time')},
                LaunchConfiguration('config_land')],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='goto_behaviour',
            executable='goto_behaviour_node',
            namespace=LaunchConfiguration('drone_id'),
            parameters=[
                {"use_sim_time": LaunchConfiguration('use_sim_time')},
                LaunchConfiguration('config_goto')],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='follow_path_behaviour',
            executable='follow_path_behaviour_node',
            namespace=LaunchConfiguration('drone_id'),
            parameters=[
                {"use_sim_time": LaunchConfiguration('use_sim_time')},
                LaunchConfiguration('config_follow_path')],
            output='screen',
            emulate_tty=True
        ),

        Node(
            package='as2_platform_behaviors',
            executable='set_arming_state_behavior',
            namespace=LaunchConfiguration('drone_id'),
            parameters=[
                {"use_sim_time": LaunchConfiguration('use_sim_time')}],
            output='screen',
            emulate_tty=True
       ),
        Node(
            package='as2_platform_behaviors',
            executable='set_offboard_mode_behavior',
            namespace=LaunchConfiguration('drone_id'),
            parameters=[
                {"use_sim_time": LaunchConfiguration('use_sim_time')}],
            output='screen',
            emulate_tty=True
        )
    ])
