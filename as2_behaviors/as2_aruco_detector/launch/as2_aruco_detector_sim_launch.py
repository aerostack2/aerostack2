from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import yaml

package_name = 'as2_aruco_detector'

def generate_launch_description():

    namespace = DeclareLaunchArgument('namespace', default_value='drone0')
    camera_image_topic = DeclareLaunchArgument('camera_image_topic', default_value='sensor_measurements/camera/image_raw')
    camera_info_topic = DeclareLaunchArgument('camera_info_topic', default_value='sensor_measurements/camera/camera_info')

    config = os.path.join(get_package_share_directory(package_name),
                          'config/'+package_name,
                          'sim_params.yaml')
    usbcam_camerainfo = os.path.join(get_package_share_directory(package_name),
                                     'config/usb_cam',
                                     'camerainfo.yaml')

    return LaunchDescription([
        namespace,
        camera_image_topic,
        camera_info_topic,

        Node(
            package=package_name,
            executable=package_name+'_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{'camera_image_topic': LaunchConfiguration('camera_image_topic'),
                         'camera_info_topic': LaunchConfiguration('camera_info_topic')},
                        config],
            output='screen',
            emulate_tty=True,
        ),
    ])
