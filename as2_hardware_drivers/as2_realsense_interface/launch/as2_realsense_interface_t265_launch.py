from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction 
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

package_name = 'as2_realsense_interface'

def generate_launch_description():
    # Declare arguments
    namespace = DeclareLaunchArgument('namespace', default_value='drone0')
    node_frequency = DeclareLaunchArgument('node_frequency', default_value='100.0')
    device = 't265'

    tf_device_config = PathJoinSubstitution([
        FindPackageShare(package_name),
        'config/tf_device',
        device+'.yaml'
    ])

    return LaunchDescription([
        namespace,
        node_frequency,
        DeclareLaunchArgument('tf_device_config', default_value=tf_device_config),

        Node(
          package=package_name,
          executable=package_name+'_node',
          namespace=LaunchConfiguration('namespace'),
          parameters=[{'node_frequency': LaunchConfiguration('node_frequency')},
                      LaunchConfiguration('tf_device_config')],
          output='screen',
          emulate_tty=True,
        )
  ])
