"""
payload.py
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


from enum import Enum
from typing import Union, List
from ign_assets.bridges.bridge import Bridge
from ign_assets.bridges import bridges as ign_bridges
from ign_assets.bridges import custom_bridges as ign_custom_bridges
from ign_assets.models.entity import Entity
from launch_ros.actions import Node


class CameraTypeEnum(str, Enum):
    """Valid camera model types"""
    VGA_CAM = 'vga_camera'
    HD_CAM = 'hd_camera'
    SEMANTIC_CAM = 'semantic_camera'

    @staticmethod
    def bridges(world_name: str, model_name: str, payload: str,
                sensor_name: str, model_prefix: str = '') -> List[Bridge]:
        """Return bridges needed for camera model

        :param world_name: gz world name
        :param model_name: gz drone model name
        :param payload: gz payload (sensor) model type
        :param sensor_name: gz payload (sensor) model name
        :param model_prefix: ros model prefix, defaults to ''
        :return: list with bridges
        """
        bridges = [
            ign_bridges.image(world_name, model_name, sensor_name,
                              payload, model_prefix),
            ign_bridges.camera_info(world_name, model_name,
                                    sensor_name, payload, model_prefix)
        ]
        return bridges


class DepthCameraTypeEnum(str, Enum):
    """Valid depth camera model types"""
    RGBD_CAM = 'rgbd_camera'

    @staticmethod
    def bridges(world_name: str, model_name: str, payload: str,
                sensor_name: str, model_prefix: str = '') -> List[Bridge]:
        """Return bridges needed for depth camera model

        :param world_name: gz world name
        :param model_name: gz drone model name
        :param payload: gz payload (sensor) model type
        :param sensor_name: gz payload (sensor) model name
        :param model_prefix: ros model prefix, defaults to ''
        :return: list with bridges
        """
        bridges = [
            ign_bridges.image(
                world_name, model_name, sensor_name, payload, model_prefix),
            ign_bridges.camera_info(
                world_name, model_name, sensor_name, payload, model_prefix),
            ign_bridges.depth_image(
                world_name, model_name, sensor_name, payload, model_prefix),
            ign_bridges.camera_points(
                world_name, model_name, sensor_name, payload, model_prefix)
        ]
        return bridges


class LidarTypeEnum(str, Enum):
    """Valid lidar model types"""
    POINT_LIDAR = 'point_lidar'  # FIXME: not working
    PLANAR_LIDAR = 'planar_lidar'
    LIDAR_3D = 'lidar_3d'

    @staticmethod
    def bridges(world_name: str, model_name: str, payload: str,
                sensor_name: str, model_prefix: str = '') -> List[Bridge]:
        """Return bridges needed for lidar model

        :param world_name: gz world name
        :param model_name: gz drone model name
        :param payload: gz payload (sensor) model type
        :param sensor_name: gz payload (sensor) model name
        :param model_prefix: ros model prefix, defaults to ''
        :return: list with bridges
        """
        bridges = [
            ign_bridges.lidar_scan(
                world_name, model_name, sensor_name, payload, model_prefix),
            ign_bridges.lidar_points(
                world_name, model_name, sensor_name, payload, model_prefix)
        ]
        return bridges


class GpsTypeEnum(str, Enum):
    """Valid GPS model types"""
    GPS = 'gps'

    @staticmethod
    def nodes(world_name: str, model_name: str, payload: str,
              sensor_name: str, model_prefix: str = '') -> List[Node]:
        """Return custom bridges (nodes) needed for gps model

        :param world_name: gz world name
        :param model_name: gz drone model name
        :param payload: gz payload (sensor) model type
        :param sensor_name: gz payload (sensor) model name
        :param model_prefix: ros model prefix, defaults to ''
        :return: list with bridges
        """
        nodes = [ign_custom_bridges.gps_node(
            world_name, model_name, sensor_name, payload)
        ]
        return nodes

    @staticmethod
    def bridges(world_name: str, model_name: str, payload: str,
                sensor_name: str, model_prefix: str = '') -> List[Bridge]:
        """Return bridges needed for gps model

        :param world_name: gz world name
        :param model_name: gz drone model name
        :param payload: gz payload (sensor) model type
        :param sensor_name: gz payload (sensor) model name
        :param model_prefix: ros model prefix, defaults to ''
        :return: list with bridges
        """
        # FIXME: current version of standard gz navsat bridge is not working properly
        bridges = [
            ign_bridges.navsat(
                world_name, model_name, sensor_name, payload, model_prefix)
        ]
        return bridges


class GripperTypeEnum(str, Enum):
    """Valid gripper model types"""
    SUCTION_GRIPPER = 'suction_gripper'

    @staticmethod
    def bridges(world_name: str, model_name: str, payload: str,
                sensor_name: str, model_prefix: str = '') -> List[Bridge]:
        """Return bridges needed for gripper model

        :param world_name: gz world name
        :param model_name: gz drone model name
        :param payload: gz payload (sensor) model type
        :param sensor_name: gz payload (sensor) model name
        :param model_prefix: ros model prefix, defaults to ''
        :return: list with bridges
        """
        bridges = [
            ign_bridges.gripper_suction_control(model_name),
            ign_bridges.gripper_contact(model_name, 'center'),
            ign_bridges.gripper_contact(model_name, 'left'),
            ign_bridges.gripper_contact(model_name, 'right'),
            ign_bridges.gripper_contact(model_name, 'top'),
            ign_bridges.gripper_contact(model_name, 'bottom')
        ]
        return bridges


class Payload(Entity):
    """Gz Payload Entity

    Use model_type as sensor_type
    """
    model_type: Union[CameraTypeEnum, DepthCameraTypeEnum, LidarTypeEnum,
                      GpsTypeEnum, GripperTypeEnum]

    def bridges(self, world_name, drone_model_name) -> tuple[List[Bridge], List[Node]]:
        """Return bridges from payload model

        :param world_name: world name
        :param drone_model_name: drone model name
        :param model_prefix: ros topic prefix name, defaults to ''
        :return ([bridges], [nodes])
        bridges -> standard bridges
        nodes -> custom bridges
        """
        bridges = []
        nodes = []

        if isinstance(self.model_type, GpsTypeEnum):
            # FIXME: current version of standard gz navsat bridge is not working properly
            # custom bridge
            nodes = self.model_type.nodes(world_name, drone_model_name, self.model_type.value,
                                          self.model_name, self.model_name)
        else:
            bridges = self.model_type.bridges(world_name, drone_model_name, self.model_type.value,
                                    self.model_name, self.model_name)
        return bridges, nodes

    def generate(self, world) -> tuple[str, str]:
        """Not model generated from payload, use drone instead"""
        return "", ""
