# Copyright 2024 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Launch as2_platform_multirotor_simulator node with a world configuration."""

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import os

from ament_index_python.packages import get_package_share_directory
from as2_core.declare_launch_arguments_from_config_file import DeclareLaunchArgumentsFromConfigFile
from as2_core.launch_configuration_from_config_file import LaunchConfigurationFromConfigFile
import as2_core.launch_param_utils as as2_utils
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def get_world_config_data(context) -> dict:
    """
    Get world configuration parameters.

    :param context: Launch context
    :type context: LaunchContext
    :return: Dict with parameter
    :rtype: dict
    """
    # Get namespace and world configuration, then search for the namespace
    # in the world configuration file and get all the parameters.
    namespace = LaunchConfiguration('namespace').perform(context)
    world_config = LaunchConfiguration('world_config').perform(context)
    yaml_data, _ = as2_utils._open_yaml_file(  # pylint: disable=protected-access
        world_config)
    yaml_data_flat = {}
    if namespace in yaml_data:
        yaml_data_flat = as2_utils._flat_dictionary(  # pylint: disable=protected-access
            yaml_data[namespace])
    return yaml_data_flat


def get_world_config_declare_launch_arguments(context) -> list:
    """
    Get world configuration declare launch arguments.

    :param context: Launch context
    :type context: LaunchContext
    :return: List with declare launch arguments
    :rtype: list
    """
    yaml_data_flat = get_world_config_data(context)
    return as2_utils._dict_to_declare_launch_argument(  # pylint: disable=protected-access
        yaml_data_flat)


def get_world_config_launch_configuration(context, *args, **kwargs) -> dict:
    """
    Get world configuration launch configuration.

    :param context: Launch context
    :type context: LaunchContext
    :return: Dict with launch configuration
    :rtype: dict
    """
    yaml_data_flat = get_world_config_data(context)
    return as2_utils._dict_to_launch_configuration(  # pylint: disable=protected-access
        yaml_data_flat)


def get_node(context, *args, **kwargs) -> list:
    """
    Get node.

    :param context: Launch context
    :type context: LaunchContext
    :return: List with node
    :rtype: list[Node]
    """
    node = Node(
        package='as2_platform_multirotor_simulator',
        executable='as2_platform_multirotor_simulator_node',
        name='platform',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        arguments=['--ros-args', '--log-level',
                   LaunchConfiguration('log_level')],
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('simulation_config'),
            LaunchConfiguration('uav_config'),
            LaunchConfigurationFromConfigFile(
                'config_file',
                default_file=kwargs['config_file_default']),
            get_world_config_launch_configuration(context),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'control_modes_file': LaunchConfiguration('control_modes_file'),
            },
        ]
    )
    return [node]


def generate_launch_description() -> LaunchDescription:
    """
    Entry point for launch file.

    :return: Launch description
    :rtype: LaunchDescription
    """
    # Get default platform configuration file
    package_folder = get_package_share_directory(
        'as2_platform_multirotor_simulator')
    config_file_default = os.path.join(package_folder,
                                       'config/platform_config_file.yaml')

    control_modes = os.path.join(package_folder,
                                 'config/control_modes.yaml')

    simulation_config = os.path.join(package_folder,
                                     'config/simulation_config.yaml')

    uav_config = os.path.join(package_folder,
                              'config/uav_config.yaml')

    world_config = os.path.join(package_folder,
                                'config/world_config.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('namespace',
                              default_value=EnvironmentVariable(
                                  'AEROSTACK2_SIMULATION_DRONE_ID'),
                              description='Drone namespace'),
        DeclareLaunchArgument('control_modes_file',
                              default_value=control_modes,
                              description='Platform control modes file'),
        DeclareLaunchArgument('uav_config',
                              default_value=uav_config,
                              description='UAV configuration file'),
        DeclareLaunchArgument('simulation_config',
                              default_value=simulation_config,
                              description='Simulation configuration file'),
        DeclareLaunchArgument('world_config',
                              default_value=world_config,
                              description='World configuration file'),
        OpaqueFunction(function=get_world_config_declare_launch_arguments),
        DeclareLaunchArgumentsFromConfigFile(
            name='config_file', source_file=config_file_default,
            description='Configuration file'),
        OpaqueFunction(function=get_node, kwargs={
                       'config_file_default': config_file_default}),
    ])
