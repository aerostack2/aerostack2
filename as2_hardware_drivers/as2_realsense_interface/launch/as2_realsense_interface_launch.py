from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

package_name = 'as2_realsense_interface'

def realsenseNode(context, *args, **kwargs):

    device_string = LaunchConfiguration('device').perform(context)
    if device_string == '': 
        print('No device specified')
        return None
    if device_string not in ['t265', 'd435','d435i']:
        print('Device not supported')
        return None

    print(f'Device selected: {device_string}')

    tf_device_config = PathJoinSubstitution([
        FindPackageShare(package_name),
        'config/tf_device',
        device_string+'.yaml'
    ])

    return [
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
  ]

def generate_launch_description():
    namespace = DeclareLaunchArgument('namespace', default_value='drone0')
    node_frequency = DeclareLaunchArgument('node_frequency', default_value='100.0')
    device = DeclareLaunchArgument('device', default_value='')

    return LaunchDescription([
        namespace,
        device,
        node_frequency,
        OpaqueFunction(function=realsenseNode)
    ])

