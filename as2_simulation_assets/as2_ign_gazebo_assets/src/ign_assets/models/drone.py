"""
drone.py
"""

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


__authors__ = "Pedro Arias Pérez"
__copyright__ = "Copyright (c) 2022 Universidad Politécnica de Madrid"
__license__ = "BSD-3-Clause"
__version__ = "0.1.0"


import os
import codecs
import subprocess
from enum import Enum
from typing import List
from pathlib import Path
from pydantic import root_validator
from ign_assets.bridges.bridge import Bridge
from ign_assets.bridges import bridges as ign_bridges
from ign_assets.bridges import custom_bridges as ign_custom_bridges
from ign_assets.models.entity import Entity
from ign_assets.models.payload import Payload
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


class DroneTypeEnum(str, Enum):
    """Valid drone model types"""
    QUADROTOR = 'quadrotor_base'
    HEXROTOR = 'hexrotor_base'


class Drone(Entity):
    """Gz Drone Entity"""
    model_type: DroneTypeEnum
    flight_time: int = 0  # in minutes
    battery_capacity: float = 0  # Ah
    payload: List[Payload] = []

    @root_validator
    def set_battery_capacity(cls, values: dict) -> dict:
        """Set battery capacity with a given flight time"""
        flight_time = float(values.get('flight_time', 0))

        # calculate battery capacity from time
        # capacity (Ah) = flight time (in hours) * load (watts) / voltage
        # assume constant voltage for battery to keep things simple for now.
        battery_capacity = (flight_time / 60) * 6.6 / 12.694
        values['battery_capacity'] = battery_capacity
        return values

    def __str__(self) -> str:
        pld_str = ""
        for pld in self.payload:
            pld_str += f" {pld}"
        return f"{super().__str__()}:{pld_str}"

    def get_index(self, world: 'World') -> int:
        """From a world.drones list which instance am I"""
        return world.drones.index(self)

    def bridges(self, world_name: str) -> tuple[List[Bridge], List[Node]]:
        """Return gz_to_ros bridges needed for the drone to fly

        :return ([bridges], [nodes])
        bridges -> standard bridges
        nodes -> custom bridges
        """
        bridges = [
            # IMU
            ign_bridges.imu(
                world_name, self.model_name, 'imu', 'internal'),
            # Magnetometer
            ign_bridges.magnetometer(
                world_name, self.model_name, 'magnetometer', 'internal'),
            # Air Pressure
            ign_bridges.air_pressure(
                world_name, self.model_name, 'air_pressure', 'internal'),
            # odom: deprecated; not used, use ground_truth instead
            # ign_bridges.odom(self.model_name),
            # pose
            ign_bridges.tf_pose(self.model_name),
            # pose static
            ign_bridges.tf_pose_static(self.model_name),
            # twist
            ign_bridges.cmd_vel(self.model_name),
            # arm
            ign_bridges.arm(self.model_name)
        ]
        if self.battery_capacity != 0:
            bridges.append(ign_bridges.battery(self.model_name))

        nodes = [
            # Odom --> ground_truth
            ign_custom_bridges.ground_truth_node(self.model_name),
            # Deprecated
            # ign_custom_bridges.tf_broadcaster_node(world_name, self.model_name)
        ]

        bridges_, nodes_ = self.payload_bridges(world_name)

        bridges.extend(bridges_)
        nodes.extend(nodes_)

        return bridges, nodes

    def payload_bridges(self, world_name: str) -> tuple[List[Bridge], List[Node]]:
        """Get bridges from payload"""
        bridges = []
        nodes = []
        for pld in self.payload:
            bridges_, nodes_ = pld.bridges(
                world_name, drone_model_name=self.model_name)
            bridges.extend(bridges_)
            nodes.extend(nodes_)

        return bridges, nodes

    def get_model_jinja_template(self) -> Path:
        """Return Path of self jinja template"""
        # Concatenate the model directory and the IGN_GAZEBO_RESOURCE_PATH environment variable
        model_dir = Path(get_package_share_directory(
            'as2_ign_gazebo_assets'), 'models')
        resource_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH')

        paths = [model_dir]
        if resource_path:
            paths += [Path(p) for p in resource_path.split(':')]

        # Define the filename to look for
        filename = f'{self.model_type}/{self.model_type}.sdf.jinja'

        # Loop through each directory and check if the file exists
        for path in paths:
            filepath = path / filename
            if filepath.is_file():
                # If the file exists, return the path
                return filepath
        raise FileNotFoundError(
            f'{filename} not found in {paths}. Does the model jinja template exists?')

    def generate(self, world) -> tuple[str, str]:
        """Generate SDF by executing JINJA and populating templates

        :raises RuntimeError: if jinja fails
        :return: python3 jinja command and path to model_sdf generated
        """

        # Concatenate the model directory and the IGN_GAZEBO_RESOURCE_PATH environment variable
        model_dir = Path(get_package_share_directory(
            'as2_ign_gazebo_assets'), 'models')
        jinja_script = os.path.join(
            get_package_share_directory('as2_ign_gazebo_assets'), 'scripts')

        payload = ""
        for pld in self.payload:
            x_s, y_s, z_s = pld.xyz
            roll_s, pitch_s, yaw_s = pld.rpy

            payload += f"{pld.model_name} {pld.model_type} {x_s} {y_s} {z_s} "
            payload += f"{roll_s} {pitch_s} {yaw_s} "

        output_file_sdf = f"/tmp/{self.model_type}_{self.get_index(world)}.sdf"
        command = ['python3', f'{jinja_script}/jinja_gen.py', self.get_model_jinja_template(),
                   f'{model_dir}/..', '--namespace', f'{self.model_name}',
                   '--sensors', f'{payload}', '--battery', f'{self.flight_time}',
                   '--output-file', f'{output_file_sdf}']

        process = subprocess.Popen(command,
                                   stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE)

        # evaluate error output to see if there were undefined variables
        # for the JINJA process
        stderr = process.communicate()[1]
        err_output = codecs.getdecoder('unicode_escape')(stderr)[0]
        for line in err_output.splitlines():
            if line.find('undefined local') > 0:
                raise RuntimeError(line)

        return command, output_file_sdf
