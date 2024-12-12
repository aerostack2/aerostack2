"""custom_bridges.py."""

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


__authors__ = 'Pedro Arias Pérez, Javier Melero Deza, Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'
__version__ = '0.1.0'

from launch_ros.actions import Node


def prefix(world_name, model_name, model_sens_name, link_name='sensor_link'):
    """Gz prefix for topics."""
    prefix = f'/world/{world_name}/model/{model_name}'
    prefix = prefix + f'/model/{model_sens_name}/link/{link_name}/sensor'
    return prefix
    # return
    # f'/world/{world_name}/model/{model_name}/model/{model_sens_name}/link/{link_name}/sensor'


def gps_node(world_name: str, namespace: str, model_sensor_name: str, link_name: str,
             use_sim_time: bool = True) -> Node:
    """Define custom GPS bridge."""
    return Node(
        package='as2_gazebo_assets',
        executable='gps_bridge',
        namespace=namespace,
        output='screen',
        parameters=[
            {'world_name': world_name,
             'name_space': namespace,
             'sensor_name': model_sensor_name,
             'link_name': link_name,
             'sensor_type': 'navsat',
             'use_sim_time': use_sim_time}
        ]
    )


def ground_truth_node(namespace: str) -> Node:
    """Define custom ground truth bridge."""
    return Node(
        package='as2_gazebo_assets',
        executable='ground_truth_bridge',
        namespace=namespace,
        output='screen',
        parameters=[
            {'name_space': namespace,
             'pose_frame_id': 'earth',
             'twist_frame_id': namespace + '/base_link'},
        ]
    )


def azimuth_node(namespace: str) -> Node:
    """Define custom azimuth bridge."""
    return Node(
        package='as2_gazebo_assets',
        executable='azimuth_bridge',
        namespace=namespace,
        output='screen',
        parameters=[
            {'name_space': namespace}
        ]
    )


def acro(namespace: str, use_sim_time: bool = True) -> Node:
    """Define custom acro bridge."""
    return Node(
        package='as2_gazebo_assets',
        executable='acro_bridge',
        namespace=namespace,
        output='screen',
        parameters=[
            {'name_space': namespace,
             'use_sim_time': use_sim_time}
        ]
    )


def set_pose_bridge(world_name: str) -> Node:
    """Set pose bridge."""
    return Node(
        package='as2_gazebo_assets',
        executable='set_entity_pose_bridge',
        output='screen',
        parameters=[
            {'world_name': world_name}
        ]
    )


def tf_broadcaster_node(world_name: str, namespace: str, parent_frame: str = 'earth',
                        use_sim_time: bool = True) -> Node:
    """
    Define custom tf broadcaster.

    This hangs tf tree built from model links from parent_frame
    """
    return Node(
        package='as2_gazebo_assets',
        executable='object_tf_broadcaster',
        namespace=namespace,
        output='screen',
        parameters=[
            {
                'world_frame': parent_frame,
                'namespace': namespace,
                'world_name': world_name,
                'use_sim_time': use_sim_time
            }
        ]
    )


def gimbal_node(world_name: str, namespace: str, model_sensor_name: str,
                gimbal_name: str, control_mode: str) -> Node:
    """
    Define custom tf broadcaster.

    This hangs tf tree built from model links from parent_frame
    """
    return Node(
        package='as2_gazebo_assets',
        executable='gimbal_bridge',
        namespace=namespace,
        output='screen',
        parameters=[
            {
                'namespace': namespace,
                'sensor_name': model_sensor_name,
                'gimbal_name': gimbal_name,
                'control_mode': control_mode,
                'world_name': world_name,
                'use_sim_time': True
            }
        ]
    )


def static_tf_node(drone_model_name: str, sensor_model_name: str,
                   sensor_model_type: str, gimbal_name: str, gimbaled: bool) -> Node:
    """
    Define static tf.

    This static tf fixes camera optical frames.
    """
    if not gimbaled:
        parent_frame = f'/{drone_model_name}/{sensor_model_name}/{sensor_model_type}/camera',
        child_frame = f'/{drone_model_name}/{sensor_model_name}/' + \
            f'{sensor_model_type}/camera/optical_frame'
    else:
        parent_frame = f'/{drone_model_name}/{gimbal_name}/_0/_1/_2/{sensor_model_name}/' + \
            f'{sensor_model_type}/camera'
        child_frame = f'/{drone_model_name}/{gimbal_name}/_0/_1/_2/{sensor_model_name}/' + \
            f'{sensor_model_type}/camera/optical_frame'

    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0', '0', '0',
            '-1.57', '0', '-1.57',
            parent_frame,
            child_frame
        ]
    )
