"""as2_visualization.launch.py."""


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
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
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


__authors__ = 'Pedro Arias Pérez'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import os
from pathlib import Path
from xml.etree import ElementTree

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def scale_model(tree, scale: float):
    """Uniform RVIZ-ONLY visual scale of the drone model.

    RViz's RobotModel display has no scale property and a sub-metre quadrotor is
    a few pixels once the camera is far enough to frame a whole mission. Scaling
    the description instead makes the drone itself readable (no proxy Axes
    marker needed).

    Multiplies every <scale> and the TRANSLATION of every <pose> (rotation
    untouched) so links, rotors and their offsets grow together — the shape does
    not break. Meshes with no explicit <scale> get one. The base_link origin is
    unchanged, so the drone still renders at its TRUE position (only its
    geometry is enlarged); nothing here reaches Gazebo, physics or the
    multirotor simulator — this description feeds robot_state_publisher/RViz
    only.
    """
    if scale == 1.0:
        return

    for mesh in tree.findall('.//mesh'):
        node = mesh.find('scale')
        if node is None:
            node = ElementTree.SubElement(mesh, 'scale')
            node.text = '1 1 1'
        node.text = ' '.join(
            f'{float(v) * scale:g}' for v in node.text.split())

    for pose in tree.findall('.//pose'):
        if not pose.text:
            continue
        parts = pose.text.split()
        if len(parts) < 3:
            continue
        xyz = [f'{float(v) * scale:g}' for v in parts[:3]]
        pose.text = ' '.join(xyz + parts[3:])


def sdf2viz(sdf_file: str, scale: float = 1.0):
    """Create SDF compatible to URDF conversion."""
    tree = ElementTree.parse(sdf_file)

    model = tree.find('./model')
    model_pose = model.find('./pose')
    if model_pose is not None:
        model.remove(model_pose)

    # Get path to the models
    sdf_base_path = os.path.dirname(os.path.dirname(sdf_file))
    # Replace all <uri> elements
    for uri in tree.findall('.//uri'):
        if uri.text.startswith('model://'):
            # Extract the relative path after 'model://'
            relative_path = uri.text.split('model://')[-1]
            # Build the absolute path
            absolute_path = os.path.join(sdf_base_path, relative_path)
            # Update the URI text
            if os.path.exists(absolute_path):
                uri.text = f'file://{absolute_path}'
            else:
                raise FileNotFoundError(f'{absolute_path} not found')

    for sensor_model in tree.findall('.//sensor/../..'):
        sensor_name = sensor_model.attrib['name']
        model.remove(sensor_model)

        for joint in model.iter('joint'):
            if joint.find('./child').text == sensor_name:
                model.remove(joint)

    # Rotor joints are 'revolute', so robot_state_publisher can only place those
    # links once /joint_states arrives — and nothing publishes it here (no
    # joint_state_publisher). RViz's RobotModel then reports
    # "No transform from [<ns>/rotor_N]" and the model shows up as an ERROR.
    # RViz never spins the rotors anyway, so pin them: 'fixed' joints get a
    # static transform straight from robot_state_publisher, no /joint_states
    # needed and no error.
    for joint in tree.findall('.//joint'):
        if joint.get('type') == 'revolute':
            joint.set('type', 'fixed')

    # RViz-only visual enlargement (see scale_model docstring).
    scale_model(tree, scale)

    # TODO(pariaspe): Avoid saving file and return directly the string
    # return ElementTree.dump(tree)

    base = os.path.basename(sdf_file)
    if scale != 1.0:
        stem, ext = os.path.splitext(base)
        base = f'{stem}_x{scale:g}{ext}'
    new_file = os.path.join('/tmp/', base)
    tree.write(new_file)
    return new_file


def get_model_path(model_name: str) -> Path:
    """Return Path of model sdf file."""
    # Concatenate the model directory and the GZ_SIM_RESOURCE_PATH environment variable
    model_dir = Path(get_package_share_directory(
        'as2_gazebo_assets'), 'models')
    resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH')

    paths = [model_dir]
    if resource_path:
        paths += [Path(p) for p in resource_path.split(':')]

    # Define the filename to look for
    filename = f'{model_name}/{model_name}.sdf'

    # Loop through each directory and check if the file exists
    for path in paths:
        filepath = path / filename
        if filepath.is_file():
            # If the file exists, return the path
            return filepath
    raise FileNotFoundError(
        f'{filename} not found in {paths}. Does the model sdf template exists?'
    )


def generate_robot_state_publisher(context: LaunchContext):
    """Publish drone URDF."""
    sdf_file = get_model_path(LaunchConfiguration('drone_model').perform(context))
    scale = float(LaunchConfiguration('viz_scale').perform(context))
    namespace = LaunchConfiguration('namespace').perform(context)

    with open(sdf2viz(sdf_file, scale), 'r', encoding='utf-8') as info:
        robot_desc = info.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time'),
             'robot_description': robot_desc,
             # The node is namespaced but TF frames are NOT namespaced by that
             # alone: without frame_prefix it emits bare 'rotor_0'..., while the
             # state estimator emits '<ns>/base_link' and RViz's RobotModel (TF
             # Prefix = <ns>) looks for '<ns>/rotor_0'. The two trees never join
             # and every non-root link reports "No transform" (model shows ERROR,
             # rotors do not render). Prefix them to match.
             'frame_prefix': f'{namespace}/'}
        ]
    )

    return [robot_state_publisher]


def generate_launch_description():
    """Publish drone URDF."""
    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # Visualization markers
    viz_markers = Node(
        package='as2_visualization',
        executable='marker_publisher',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'namespace': LaunchConfiguration('namespace'),
                'record_length': LaunchConfiguration('record_length'),
            }
        ],
        condition=IfCondition(LaunchConfiguration('paint_markers')),
    )

    # Visualization geozones
    viz_geozones = Node(
        package='as2_visualization',
        executable='geozones_marker_publisher',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'namespace': LaunchConfiguration('namespace'),
            }
        ],
        condition=IfCondition(LaunchConfiguration('paint_geozones')),
    )

    default_rviz_config = os.path.join(
        get_package_share_directory('as2_visualization'), 'config', 'as2_default.rviz'
    )

    return LaunchDescription(
        [
            # Launch Arguments
            DeclareLaunchArgument('namespace', description='Drone namespace.'),
            DeclareLaunchArgument(
                'use_sim_time', default_value='false', description='Use simulation time'
            ),
            DeclareLaunchArgument(
                'rviz', default_value='true', description='Open RViz.'
            ),
            DeclareLaunchArgument(
                'rviz_config',
                default_value=default_rviz_config,
                description='RViz configuration file.',
            ),
            DeclareLaunchArgument(
                'drone_model',
                default_value='quadrotor_base',
                description='Drone model to visualize. Available options: quadrotor_base,'
                ' hexrotor_base, crazyflie, x500, px4vision, or use your own model.',
            ),
            DeclareLaunchArgument(
                'viz_scale',
                default_value='5.0',
                description='RVIZ-ONLY visual scale of the drone model. A sub-metre'
                ' quadrotor is a few pixels once the camera frames a whole mission'
                ' (200x200 m grid), so the model is enlarged. Position stays TRUE'
                ' (only the geometry grows); Gazebo/physics are untouched. 5.0 =>'
                ' ~2.5 m drone: 1/4 of a 10 m grid cell, no overlap at the 5 m swarm'
                ' spawn spacing. Set 1.0 for true scale.',
            ),
            DeclareLaunchArgument(
                'paint_markers',
                default_value='true',
                choices=['false', 'true'],
                description='Publish markers to paint reference pose, vel and poses history.',
            ),
            DeclareLaunchArgument(
                'paint_geozones',
                default_value='false',
                choices=['false', 'true'],
                description='Marker publisher subscribes to geofence topics.',
            ),
            DeclareLaunchArgument(
                'record_length',
                default_value='500',
                description='Length for last poses.',
            ),
            OpaqueFunction(function=generate_robot_state_publisher),
            SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
            rviz,
            viz_markers,
            viz_geozones,
        ]
    )
