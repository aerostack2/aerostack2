from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes, Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    container = ComposableNodeContainer(
        name='container',   # Nombre container donde se indexa
        namespace='drone0',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
                ComposableNode(
                    package='as2_alphanumeric_viewer',
                    plugin='AlphanumericViewer', #############
                    name='AlphanumericViewer', #############
                    # executable='as2_alphanumeric_viewer_node',
                    namespace='drone0',
                    parameters=[{
                    'namespace':'drone0',
                    'use_sim_time': False
        }]
                    # output='screen',
                    # emulate_tty=True
                )],
            output='both'
        )

    launch_description = LaunchDescription([container])

    return launch_description


# def generate_launch_description():
#     return LaunchDescription([
#         DeclareLaunchArgument('drone_id', default_value='drone0'),
#         DeclareLaunchArgument('use_sim_time', default_value='false'),
#         Node(
#             package='as2_alphanumeric_viewer',
#             executable='as2_alphanumeric_viewer_node',
#             name='alphanumeric_viewer',
#             namespace=LaunchConfiguration('drone_id'),
#             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
#             output='screen',
#             emulate_tty=True
#         )
#     ])
