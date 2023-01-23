"""Keyboard Teleopration launch."""

# Copyright 2022 Universidad Politécnica de Madrid
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

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction


def launch_teleop(context):
    """Teleop python process."""
    keyboard_teleop = os.path.join(get_package_share_directory(
        'as2_keyboard_teleoperation'), 'keyboard_teleoperation.py')

    namespace = LaunchConfiguration('namespace').perform(context)
    verbose = LaunchConfiguration('verbose').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    process = ExecuteProcess(
        cmd=['python3', keyboard_teleop, namespace, verbose, use_sim_time],
        name='as2_keyboard_teleoperation',
        output='screen')
    return [process]


def generate_launch_description():
    """Entrypoint launch description method."""
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'namespace',
            description='Drone id.'),
        DeclareLaunchArgument(
            'verbose',
            default_value='false',
            choices=['true', 'false'],
            description='Launch in verbose mode.'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Use simulation time.'),
        OpaqueFunction(function=launch_teleop),
    ])
