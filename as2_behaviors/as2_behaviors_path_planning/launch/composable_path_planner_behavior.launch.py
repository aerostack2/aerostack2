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
#    * Neither the name of the the copyright holder nor the names of its
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

"""composable_path_planner_behavior launch file."""

from as2_core.launch_plugin_utils import get_available_plugins
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Launch path planner composable node."""
    path_planner_node = ComposableNode(
        namespace=LaunchConfiguration('namespace'),
        package='as2_behaviors_path_planning',
        plugin='PathPlannerBehavior',
        name='PathPlannerBehavior',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=EnvironmentVariable(
            'AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgument(
            name='container', default_value='',
            description=(
                'Name of an existing node container to load launched nodes into. '
                'If unset, a new container will be created with name "behaviors".'
            )
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('plugin_name', description='Plugin name',
                              choices=get_available_plugins('as2_behaviors_path_planning')),
        # Load in existing container
        LoadComposableNodes(
            condition=LaunchConfigurationNotEquals('container', ''),
            composable_node_descriptions=[path_planner_node],
            target_container=(LaunchConfiguration('namespace'),
                              '/', LaunchConfiguration('container')),
        ),
        # Or create new container
        ComposableNodeContainer(
            condition=LaunchConfigurationEquals('container', ''),
            name='behaviors',  # TODO: use other name?
            namespace=LaunchConfiguration('namespace'),
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                path_planner_node
            ],
            output='screen',
            arguments=['--ros-args', '--log-level',
                       LaunchConfiguration('log_level')],
            parameters=[{
                'plugin_name': LaunchConfiguration('plugin_name'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            emulate_tty=True
        )
    ])
