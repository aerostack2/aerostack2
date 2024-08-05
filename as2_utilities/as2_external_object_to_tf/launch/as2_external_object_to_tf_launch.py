#!/usr/bin/env python3

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

"""Launch file for the as2_external_object_to_tf node."""

__authors__ = 'Javilinos'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='/'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'config_file',
            description='Source data to TF config file. Example in node config folder'),
        DeclareLaunchArgument(
            'mocap_topic',
            default_value='/mocap/rigid_bodies',
            description='Mocap topic to get external object pose'),
        Node(
            package='as2_external_object_to_tf',
            executable='as2_external_object_to_tf_node',
            name='external_object_to_tf',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                        {'config_file': LaunchConfiguration('config_file')},
                        {'mocap_topic': LaunchConfiguration('mocap_topic')}],
            output='screen',
            emulate_tty=True
        )
    ])
