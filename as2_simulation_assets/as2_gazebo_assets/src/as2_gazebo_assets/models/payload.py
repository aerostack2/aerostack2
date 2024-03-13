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

from __future__ import annotations

__authors__ = "Pedro Arias Pérez"
__copyright__ = "Copyright (c) 2022 Universidad Politécnica de Madrid"
__license__ = "BSD-3-Clause"
__version__ = "0.1.0"


from enum import Enum
from typing import Union, List
from pydantic import validator
from launch_ros.actions import Node
from as2_gazebo_assets.bridges.bridge import Bridge
from as2_gazebo_assets.bridges import bridges as gz_bridges
from as2_gazebo_assets.bridges import custom_bridges as gz_custom_bridges
from as2_gazebo_assets.models.entity import Entity


class CameraTypeEnum(str, Enum):
    """Valid camera model types"""

    VGA_CAM = "vga_camera"
    HD_CAM = "hd_camera"
    SEMANTIC_CAM = "semantic_camera"

    @staticmethod
    def bridges(
        world_name: str,
        model_name: str,
        payload: str,
        sensor_name: str,
        model_prefix: str = "",
    ) -> List[Bridge]:
        """Return bridges needed for camera model

        :param world_name: gz world name
        :param model_name: gz drone model name
        :param payload: gz payload (sensor) model type
        :param sensor_name: gz payload (sensor) model name
        :param model_prefix: ros model prefix, defaults to ''
        :return: list with bridges
        """
        bridges = [
            gz_bridges.image(world_name, model_name, sensor_name,
                             payload, model_prefix),
            gz_bridges.camera_info(world_name, model_name,
                                   sensor_name, payload, model_prefix)
        ]
        return bridges


class DepthCameraTypeEnum(str, Enum):
    """Valid depth camera model types"""

    RGBD_CAM = "rgbd_camera"

    @staticmethod
    def bridges(
        world_name: str,
        model_name: str,
        payload: str,
        sensor_name: str,
        model_prefix: str = "",
    ) -> List[Bridge]:
        """Return bridges needed for depth camera model

        :param world_name: gz world name
        :param model_name: gz drone model name
        :param payload: gz payload (sensor) model type
        :param sensor_name: gz payload (sensor) model name
        :param model_prefix: ros model prefix, defaults to ''
        :return: list with bridges
        """
        bridges = [
            gz_bridges.image(
                world_name, model_name, sensor_name, payload, model_prefix),
            gz_bridges.camera_info(
                world_name, model_name, sensor_name, payload, model_prefix),
            gz_bridges.depth_image(
                world_name, model_name, sensor_name, payload, model_prefix),
            gz_bridges.camera_points(
                world_name, model_name, sensor_name, payload, model_prefix)
        ]
        return bridges


class LidarTypeEnum(str, Enum):
    """Valid lidar model types"""

    POINT_LIDAR = "point_lidar"  # FIXME: not working
    PLANAR_LIDAR = "planar_lidar"
    LIDAR_3D = "lidar_3d"

    @staticmethod
    def bridges(
        world_name: str,
        model_name: str,
        payload: str,
        sensor_name: str,
        model_prefix: str = "",
    ) -> List[Bridge]:
        """Return bridges needed for lidar model

        :param world_name: gz world name
        :param model_name: gz drone model name
        :param payload: gz payload (sensor) model type
        :param sensor_name: gz payload (sensor) model name
        :param model_prefix: ros model prefix, defaults to ''
        :return: list with bridges
        """
        bridges = [
            gz_bridges.lidar_scan(
                world_name, model_name, sensor_name, payload, model_prefix),
            gz_bridges.lidar_points(
                world_name, model_name, sensor_name, payload, model_prefix)
        ]
        return bridges


class GpsTypeEnum(str, Enum):
    """Valid GPS model types"""

    GPS = "gps"

    @staticmethod
    def nodes(
        world_name: str,
        model_name: str,
        payload: str,
        sensor_name: str,
        model_prefix: str = "",
    ) -> List[Node]:
        """Return custom bridges (nodes) needed for gps model

        :param world_name: gz world name
        :param model_name: gz drone model name
        :param payload: gz payload (sensor) model type
        :param sensor_name: gz payload (sensor) model name
        :param model_prefix: ros model prefix, defaults to ''
        :return: list with bridges
        """
        nodes = [gz_custom_bridges.gps_node(
            world_name, model_name, sensor_name, payload)
        ]
        return nodes

    @staticmethod
    def bridges(
        world_name: str,
        model_name: str,
        payload: str,
        sensor_name: str,
        model_prefix: str = "",
    ) -> List[Bridge]:
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
            gz_bridges.navsat(
                world_name, model_name, sensor_name, payload, model_prefix)
        ]
        return bridges


class GripperTypeEnum(str, Enum):
    """Valid gripper model types"""

    SUCTION_GRIPPER = "suction_gripper"

    @staticmethod
    def bridges(
        world_name: str,
        model_name: str,
        payload: str,
        sensor_name: str,
        model_prefix: str = "",
    ) -> List[Bridge]:
        """Return bridges needed for gripper model

        :param world_name: gz world name
        :param model_name: gz drone model name
        :param payload: gz payload (sensor) model type
        :param sensor_name: gz payload (sensor) model name
        :param model_prefix: ros model prefix, defaults to ''
        :return: list with bridges
        """
        bridges = [
            gz_bridges.gripper_suction_control(model_name),
            gz_bridges.gripper_contact(model_name, 'center'),
            gz_bridges.gripper_contact(model_name, 'left'),
            gz_bridges.gripper_contact(model_name, 'right'),
            gz_bridges.gripper_contact(model_name, 'top'),
            gz_bridges.gripper_contact(model_name, 'bottom')
        ]
        return bridges


class GimbalTypeEnum(str, Enum):
    """Valid gimbal model types"""

    GIMBAL_SPEED = "gimbal_speed"
    GIMBAL_POSITION = "gimbal_position"

    @staticmethod
    def nodes(
        world_name: str,
        model_name: str,
        sensor_name: str,
        gimbal_name: str,
        model_type: str,
    ) -> List[Node]:
        """Return custom bridges (nodes) needed for gps model

        :param world_name: gz world name
        :param model_name: gz drone model name
        :param payload: gz payload (sensor) model type
        :param sensor_name: gz payload (sensor) model name
        :param model_prefix: ros model prefix, defaults to ''
        :return: list with bridges
        """

        gimbal_type = (
            "position"
            if model_type == GimbalTypeEnum.GIMBAL_POSITION.value
            else "speed"
        )

        nodes = [
            gz_custom_bridges.gimbal_node(
                world_name, model_name, sensor_name, gimbal_name, gimbal_type
            )
        ]

        return nodes


class Payload(Entity):
    """Gz Payload Entity

    Use model_type as sensor_type
    """

    model_type: Union[
        CameraTypeEnum, DepthCameraTypeEnum, LidarTypeEnum, GpsTypeEnum, GimbalTypeEnum
    ] = None
    sensor_attached: str = "None"
    payload: Payload = None
    gimbaled: bool = False

    @validator("payload", always=True)
    def set_gimbaled_default(cls, v, values):
        if "model_type" in values and isinstance(values["model_type"], GimbalTypeEnum):
            if v is not None:
                values["sensor_attached"] = v.model_name
                v.gimbaled = True
            else:
                raise ValueError(
                    f"{values['model_name']}({values['model_type']}): Missing field 'payload'"
                )
        else:
            if v is not None:
                raise ValueError(
                    f"{values['model_name']}({values['model_type']}): Invalid payload field"
                )
        return v

    # class Config:
    #     validate_assignment = True

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
            nodes = self.model_type.nodes(
                world_name, drone_model_name, self.model_type.value, self.model_name
            )

        elif isinstance(self.model_type, GimbalTypeEnum):
            nodes = self.model_type.nodes(
                world_name,
                drone_model_name,
                self.sensor_attached,
                self.model_name,
                self.model_type.value,
            )

        else:
            sensor_name = self.model_type.value
            if self.gimbaled:
                sensor_name = "gb/model/_0/model/_1/model/_2/model/" + self.model_type.value

            bridges = self.model_type.bridges(world_name, drone_model_name,
                                              self.model_name, sensor_name, self.model_name)
        return bridges, nodes

    def generate(self, world) -> tuple[str, str]:
        """Not model generated from payload, use drone instead"""
        return "", ""
