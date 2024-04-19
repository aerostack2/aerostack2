# Copyright 2023 Universidad Politécnica de Madrid
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
"""Launch psdk_wrapper node."""
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import EmitEvent, DeclareLaunchArgument
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
import lifecycle_msgs.msg
import launch


def generate_launch_description():
    # Declare the namespace launch argument
    psdk_params_file = PathJoinSubstitution([
        FindPackageShare('as2_platform_dji_psdk'),
        'config', 'psdk_params.yaml'
    ])

    link_config_file = PathJoinSubstitution([
        FindPackageShare('as2_platform_dji_psdk'),
        'config', 'link_config.json'
    ])

    hms_return_codes_file = PathJoinSubstitution([
        FindPackageShare('as2_platform_dji_psdk'),
        'config', 'hms_2023_08_22.json'
    ])

    # Prepare the wrapper node
    wrapper_node = LifecycleNode(
        package="psdk_wrapper",
        executable="psdk_wrapper_node",
        name="psdk_wrapper_node",
        namespace=LaunchConfiguration('namespace'),
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "link_config_file_path": LaunchConfiguration('link_config_file_path'),
                "hms_return_codes_path": LaunchConfiguration('hms_return_codes_path'),
                "tf_frame_prefix": LaunchConfiguration('tf_frame_prefix'),
            },
            LaunchConfiguration('psdk_params_file_path'),
        ],
        remappings=[
            ("psdk_ros2/gps_position_fused", "sensor_measurements/gps"),
            ("psdk_ros2/imu", "sensor_measurements/imu"),
            ("psdk_ros2/main_camera_stream", "sensor_measurements/main_camera/image_raw"),
        ]
    )

    # Configure lifecycle node
    wrapper_configure_trans_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(wrapper_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Activate lifecycle node
    wrapper_activate_trans_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(wrapper_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )

    # Create LaunchDescription and populate
    ld = LaunchDescription([
        DeclareLaunchArgument('namespace',
                              default_value=EnvironmentVariable(
                                  'AEROSTACK2_SIMULATION_DRONE_ID'),
                              description='Drone namespace'),
        DeclareLaunchArgument('psdk_params_file_path',
                              default_value=psdk_params_file,
                              description='DJI PSDK configuration file'),
        DeclareLaunchArgument('link_config_file_path',
                              default_value=link_config_file,
                              description='DJI PSDK link configuration file'),
        DeclareLaunchArgument('hms_return_codes_path',
                              default_value=hms_return_codes_file,
                              description='Path to JSON file with known DJI return codes'),
        DeclareLaunchArgument('tf_frame_prefix',
                              default_value="",
                              description='TF frame prefix'),
    ])

    # Declare Launch options
    ld.add_action(wrapper_node)
    ld.add_action(wrapper_configure_trans_event)
    ld.add_action(wrapper_activate_trans_event)

    return ld
