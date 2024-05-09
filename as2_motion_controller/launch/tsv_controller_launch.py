# # import os
# # import logging
# # from typing import List
# # from xml.etree import ElementTree
# # from launch import LaunchDescription
# # from launch.actions import DeclareLaunchArgument, OpaqueFunction
# # from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindPackageShare
# # from launch_ros.descriptions import ComposableNode
# # from ament_index_python.packages import get_package_share_directory
# # from launch_ros.actions import ComposableNodeContainer

# # FORMAT = '[%(levelname)s] [launch]: %(message)s'
# # logging.basicConfig(format=FORMAT)


# # def get_available_plugins(package_name: str) -> List[str]:
# #     """
# #     Parse plugins.xml file from package and return a list of plugins from a specific type
# #     """
# #     plugins_file = os.path.join(
# #         get_package_share_directory(package_name),
# #         'plugins.xml'
# #     )
# #     root = ElementTree.parse(plugins_file).getroot()

# #     available_plugins = []

# #     Check if the root element is a <class_libraries> or <library> tag
# #     if root.tag == 'class_libraries':
# #         Find all elements with the tag 'library' under the root
# #         libraries = root.findall('library')
# #     elif root.tag == 'library':
# #         If the root is a single <library> tag, consider it as a list itself
# #         libraries = [root]
# #     else:
# #         If the root tag is neither <class_libraries> nor <library>, return empty list
# #         return available_plugins

# #     for library in libraries:
# #         Extract plugin information from the 'class' tag
# #         classes = library.findall('class')
# #         for plugin_class in classes:
# #             plugin_type = plugin_class.attrib.get('type')
# #             if plugin_type:
# #                 plugin_name = plugin_type.split('::')[0]
# #                 available_plugins.append(plugin_name)

# #     return available_plugins


# # def generate_controller_node() -> ComposableNode:
# #     """ Returns the controller manager node """
# #     parameters = [{
# #         'plugin_name': LaunchConfiguration('plugin_name'),
# #         'use_sim_time': LaunchConfiguration('use_sim_time'),
# #         'plugin_available_modes_config_file': LaunchConfiguration(
# #             'plugin_available_modes_config_file')
# #     }]

# #     plugin_config_file = LaunchConfiguration(
# #         'plugin_config_file')

# #     if not plugin_config_file:
# #         plugin_config_file = PathJoinSubstitution([
# #             FindPackageShare('as2_motion_controller'),
# #             'plugins/', LaunchConfiguration('plugin_name'), '/config/controller_default.yaml'
# #         ])

# #     parameters.append(plugin_config_file)

# #     controller_config_file = LaunchConfiguration(
# #         'motion_controller_config_file')

# #     if not controller_config_file:
# #         controller_config_file = PathJoinSubstitution([
# #             FindPackageShare('as2_motion_controller'),
# #             'config', 'motion_controller_default.yaml'
# #         ])

# #     parameters.append(controller_config_file)

# #     return ComposableNode(
# #         package='as2_motion_controller',
# #         plugin='MotionController',
# #         namespace=LaunchConfiguration('namespace'),
# #         name='MotionController',
# #         parameters=[{
# #         'plugin_name': LaunchConfiguration('plugin_name'),
# #         'use_sim_time': LaunchConfiguration('use_sim_time'),
# #         'plugin_available_modes_config_file': LaunchConfiguration(
# #             'plugin_available_modes_config_file')
# #         }],
# #         output='screen',
# #        emulate_tty=True
# #     )


# # def generate_launch_description() -> LaunchDescription:
# #     """ Returns the launch description """
# #     controller_components = []
# #     ld = LaunchDescription()

# #     ld.add_action(DeclareLaunchArgument('namespace'))
# #     ld.add_action(DeclareLaunchArgument(
# #         'motion_controller_config_file', default_value=''))
# #     ld.add_action(DeclareLaunchArgument(
# #         'plugin_name', choices=get_available_plugins('as2_motion_controller')))
# #     ld.add_action(DeclareLaunchArgument(
# #         'plugin_config_file', default_value=''))
# #     ld.add_action(DeclareLaunchArgument(
# #         'plugin_available_modes_config_file', default_value=''))
# #     ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false'))

# #     ld.add_action(generate_controller_node())

# #     controller_components.append(generate_controller_node())

# #     container = ComposableNodeContainer(
# #         name='hola',
# #         namespace=LaunchConfiguration('namespace'),
# #         package='rclcpp_components',
# #         executable='component_container',
# #         composable_node_descriptions=controller_components,
# #         output='screen',
# #     )

# #     ld.add_action(container)

# #     return ld

# """ Launch file for the controller manager node """

# import os
# import sys
# import logging
# from typing import List
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, OpaqueFunction
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
# from ament_index_python.packages import get_package_share_directory
# from xml.etree import ElementTree
# import launch
# from launch_ros.actions import ComposableNodeContainer
# from launch_ros.descriptions import ComposableNode

# FORMAT = '[%(levelname)s] [launch]: %(message)s'
# logging.basicConfig(format=FORMAT)

# parameters = []


# def get_available_plugins(package_name: str) -> List[str]:
#     """
#     Parse plugins.xml file from package and return a list of plugins from a specific type
#     """
#     plugins_file = os.path.join(
#         get_package_share_directory(package_name),
#         'plugins.xml'
#     )
#     root = ElementTree.parse(plugins_file).getroot()

#     available_plugins = []

#     # Check if the root element is a <class_libraries> or <library> tag
#     if root.tag == 'class_libraries':
#         # Find all elements with the tag 'library' under the root
#         libraries = root.findall('library')
#     elif root.tag == 'library':
#         # If the root is a single <library> tag, consider it as a list itself
#         libraries = [root]
#     else:
#         # If the root tag is neither <class_libraries> nor <library>, return empty list
#         return available_plugins

#     for library in libraries:
#         # Extract plugin information from the 'class' tag
#         classes = library.findall('class')
#         for plugin_class in classes:
#             plugin_type = plugin_class.attrib.get('type')
#             if plugin_type:
#                 plugin_name = plugin_type.split('::')[0]
#                 available_plugins.append(plugin_name)

#     return available_plugins


# def get_controller_manager_node(context):
#     """ Returns the controller manager node """
#     global parameters

#     plugin_name = LaunchConfiguration('plugin_name').perform(context)
#     if not plugin_name:
#         logging.critical("Plugin not set.")
#         sys.exit(1)

#     parameters = [{
#         'plugin_name': plugin_name,
#         'use_sim_time': LaunchConfiguration('use_sim_time'),
#         'plugin_available_modes_config_file': LaunchConfiguration(
#             'plugin_available_modes_config_file')
#     }]

#     plugin_config_file = LaunchConfiguration(
#         'plugin_config_file').perform(context)

#     if not plugin_config_file:
#         print("Finding default config file for plugin: " + plugin_name)
#         plugin_config_file = PathJoinSubstitution([
#             FindPackageShare('as2_motion_controller'),
#             'plugins/' + plugin_name + '/config', 'controller_default.yaml'
#         ])
#         print("Found default config file: plugins/" +
#               plugin_name + "/config/controller_default.yaml")

#     parameters.append(plugin_config_file)

#     controller_config_file = LaunchConfiguration(
#         'motion_controller_config_file').perform(context)

#     if not controller_config_file:
#         controller_config_file = PathJoinSubstitution([
#             FindPackageShare('as2_motion_controller'),
#             'config', 'motion_controller_default.yaml'
#         ])

#     parameters.append(controller_config_file)


#     node = ComposableNode(
#         package='as2_motion_controller',
#         executable='as2_motion_controller_node',
#         namespace=LaunchConfiguration('namespace'),
#         parameters=parameters,
#         output='screen',
#         emulate_tty=True
#     )

#     return [node]


# def generate_launch_description():
#     """ Returns the launch description """
#     global parameters

#     ld = LaunchDescription()

#     ld.add_action(DeclareLaunchArgument(
#         'motion_controller_config_file', default_value=''))
#     ld.add_action(DeclareLaunchArgument(
#         'plugin_name', choices=get_available_plugins('as2_motion_controller')))
#     ld.add_action(DeclareLaunchArgument('plugin_config_file', default_value=''))
#     ld.add_action(DeclareLaunchArgument(
#         'plugin_available_modes_config_file', default_value=''))
#     ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false'))
#     # ld.add_action(OpaqueFunction(function=get_controller_manager_node))

#     container = ComposableNodeContainer(
#             name='hola',
#             namespace=DeclareLaunchArgument('namespace'),
#             package='rclcpp_components',
#             executable='component_container',
#             composable_node_descriptions=[
#                 ComposableNode(
#                     package='as2_motion_controller',
#                     executable='as2_motion_controller_node',
#                     namespace=LaunchConfiguration('namespace'),
#                     parameters=parameters,
#                     output='screen',
#                     emulate_tty=True)],
#             output='screen'
#     )
#     return LaunchDescription([ld, container, load_composable_nodes])


""" Launch file for the controller manager node """

import os
import sys
import logging
from typing import List

from launch_ros.actions import LoadComposableNodes, Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from xml.etree import ElementTree

FORMAT = '[%(levelname)s] [launch]: %(message)s'
logging.basicConfig(format=FORMAT)
parameters = []

def get_available_plugins(package_name: str) -> List[str]:
    """
    Parse plugins.xml file from package and return a list of plugins from a specific type
    """
    plugins_file = os.path.join(
        get_package_share_directory(package_name),
        'plugins.xml'
    )
    root = ElementTree.parse(plugins_file).getroot()

    available_plugins = []

    # Check if the root element is a <class_libraries> or <library> tag
    if root.tag == 'class_libraries':
        # Find all elements with the tag 'library' under the root
        libraries = root.findall('library')
    elif root.tag == 'library':
        # If the root is a single <library> tag, consider it as a list itself
        libraries = [root]
    else:
        # If the root tag is neither <class_libraries> nor <library>, return empty list
        return available_plugins

    for library in libraries:
        # Extract plugin information from the 'class' tag
        classes = library.findall('class')
        for plugin_class in classes:
            plugin_type = plugin_class.attrib.get('type')
            if plugin_type:
                plugin_name = plugin_type.split('::')[0]
                available_plugins.append(plugin_name)

    return available_plugins


def get_controller_manager_node(context):
    """ Returns the controller manager node """
    global parameters
    plugin_name = LaunchConfiguration('plugin_name').perform(context)
    if not plugin_name:
        logging.critical("Plugin not set.")
        sys.exit(1)

    parameters = [{
        'plugin_name': plugin_name,
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'plugin_available_modes_config_file': LaunchConfiguration(
            'plugin_available_modes_config_file')
    }]

    plugin_config_file = LaunchConfiguration(
        'plugin_config_file').perform(context)

    if not plugin_config_file:
        print("Finding default config file for plugin: " + plugin_name)
        plugin_config_file = PathJoinSubstitution([
            FindPackageShare('as2_motion_controller'),
            'plugins/' + plugin_name + '/config', 'controller_default.yaml'
        ])
        print("Found default config file: plugins/" +
              plugin_name + "/config/controller_default.yaml")

    parameters.append(plugin_config_file)

    controller_config_file = LaunchConfiguration(
        'motion_controller_config_file').perform(context)

    if not controller_config_file:
        controller_config_file = PathJoinSubstitution([
            FindPackageShare('as2_motion_controller'),
            'config', 'motion_controller_default.yaml'
        ])

    parameters.append(controller_config_file)


    # node = Node(
    #     package='as2_motion_controller',
    #     executable='as2_motion_controller_node',
    #     namespace=LaunchConfiguration('namespace'),
    #     parameters=parameters,
    #     output='screen',
    #     emulate_tty=True
    # )

    return []


def generate_launch_description():
    """ Returns the launch description """

    plugin_config_file = PathJoinSubstitution([
        FindPackageShare('as2_motion_controller'),
        'plugins/' + 'ControllerManager' + '/config', 'controller_default.yaml'
    ])


    controller_config_file = PathJoinSubstitution([
        FindPackageShare('as2_motion_controller'),
        'config', 'motion_controller_default.yaml'
    ])

        # DeclareLaunchArgument('namespace'),
        # DeclareLaunchArgument(
        #     'motion_controller_config_file', default_value=''),
        # DeclareLaunchArgument(
        #     'plugin_name', choices=get_available_plugins('as2_motion_controller')),
        # DeclareLaunchArgument('plugin_config_file', default_value=''),
        # DeclareLaunchArgument(
        #     'plugin_available_modes_config_file', default_value=''),
        # DeclareLaunchArgument('use_sim_time', default_value='false'),
        # OpaqueFunction(function=get_controller_manager_node),

    container = ComposableNodeContainer(
        name='container',   # Nombre container donde se indexa
        namespace='drone0',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
                ComposableNode(
                    package='as2_motion_controller',
                    plugin='ControllerManager',
                    name='ControllerManager',
                    # executable='as2_motion_controller_node',
                    namespace='drone0',
                    parameters=[{
                    'namespace':'drone0',
                    'plugin_name': 'pid_speed_controller',
                    'use_sim_time': True,
                    'motion_controller_config_file': controller_config_file,
                    'plugin_config_file': plugin_config_file,
                    'plugin_available_modes_config_file':'sim_config/motion_controller_plugin.yaml'
                    }]
                    # output='screen',
                    # emulate_tty=True
                )],
            output='both'
        )

    launch_description = LaunchDescription([container])

    return launch_description
