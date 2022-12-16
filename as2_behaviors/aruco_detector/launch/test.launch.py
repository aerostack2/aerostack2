from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('aruco_detector'),
        'config/aruco_detector', 'real_params.yaml'
    ])
    camera_info = PathJoinSubstitution([
        FindPackageShare('usb_camera_interface'),
        'config/usb_camera_interface', 'camera_info.yaml'
    ])
    camera_params = PathJoinSubstitution([
        FindPackageShare('usb_camera_interface'),
        'config/usb_camera_interface', 'params.yaml'
    ])
    tf_cam_config = PathJoinSubstitution([
        FindPackageShare('usb_camera_interface'),
        'config/tf_cam', 'real_arguments.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value=EnvironmentVariable('AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument('tf_cam_config', default_value=tf_cam_config),
        DeclareLaunchArgument('camera_info', default_value=camera_info),
        DeclareLaunchArgument('camera_params', default_value=camera_params),
        DeclareLaunchArgument('log_level', default_value='info'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('usb_camera_interface'),
                'launch', 'new_usb_camera_interface_launch.py'
            ])]),
            launch_arguments={
                'drone_id': LaunchConfiguration('drone_id'),
                'camera_info': LaunchConfiguration('camera_info'),
                'camera_params': LaunchConfiguration('camera_params'),
                'tf_cam_config': LaunchConfiguration('tf_cam_config')
            }.items()
        ),
        
        Node(
            package='aruco_detector',
            executable='aruco_detector_node',
            namespace=LaunchConfiguration('drone_id'),
            parameters=[LaunchConfiguration('config')],
            output='screen',
            emulate_tty=True,
            remappings=[("sensor_measurements/camera/image_raw", "sensor_measurements/aruco_camera")]
        )
    ])
