from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import yaml

package_name = 'as2_usb_camera_interface'

def generate_launch_description():
    camera_info = PathJoinSubstitution([
        FindPackageShare(package_name),
        'config/'+package_name, 'wide03_720_info.yaml'
    ])
    camera_params = PathJoinSubstitution([
        FindPackageShare(package_name),
        'config/'+package_name, 'params.yaml'
    ])
    tf_cam_config = PathJoinSubstitution([
        FindPackageShare(package_name),
        'config/tf_cam', 'f330_arguments.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='drone0'),
        DeclareLaunchArgument('camera_info', default_value=camera_info),
        DeclareLaunchArgument('camera_params', default_value=camera_params),
        DeclareLaunchArgument('tf_cam_config', default_value=tf_cam_config),
        DeclareLaunchArgument('log_level', default_value='info'),
        Node(
            package=package_name,
            executable=package_name+'_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[LaunchConfiguration('camera_params'),
                        LaunchConfiguration('camera_info'),
                        LaunchConfiguration('tf_cam_config')],
            output='screen',
            emulate_tty=True,
        ),

        # OpaqueFunction(function=staticTransformNode)
    ])
