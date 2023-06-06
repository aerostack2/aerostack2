"""
follow_reference_module.py
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


__authors__ = "Pedro Arias Pérez, Miguel Fernández Cortizas, David Pérez Saura, Rafael Pérez Seguí, Javier Melero Deza"
__copyright__ = "Copyright (c) 2022 Universidad Politécnica de Madrid"
__license__ = "BSD-3-Clause"
__version__ = "0.1.0"

from typing import List, TYPE_CHECKING

from as2_msgs.msg import YawMode
from geometry_msgs.msg import Pose

from as2_python_api.modules.module_base import ModuleBase
from as2_python_api.behavior_actions.follow_reference_behavior import FollowReferenceBehavior

if TYPE_CHECKING:
    from ..drone_interface import DroneInterface


class FollowReferenceModule(ModuleBase, FollowReferenceBehavior):
    """Follow reference Module
    """
    __alias__ = "follow_reference"

    def __init__(self, drone: 'DroneInterface') -> None:
        super().__init__(drone, self.__alias__)

    def __call__(self, _x: float, _y: float, _z: float, frame_id: str,
              speed_x: float = 0.0, speed_y: float = 0.0, speed_z: float = 0.0, yaw_mode: int = YawMode.KEEP_YAW, 
              yaw_angle: float = None, wait: bool = False) -> None:
        """Follow reference (m) with speed (m/s).

        :type _x: float
        :type _y: float
        :type _z: float
        :type frame_id: str
        :type speed_x: float
        :type speed_y: float
        :type speed_z: float
        :type yaw_mode: int
        :type yaw_angle: float
        :type wait: bool
        """
        self.__follow_reference(_x, _y, _z, frame_id, speed_x, speed_y, speed_z, yaw_mode, yaw_angle, wait)

    def __follow_reference(self, _x: float, _y: float, _z: float, frame_id: str,
                speed_x: float, speed_y: float, speed_z: float, yaw_mode: int, yaw_angle: float, wait: bool = False) -> None:
        msg = Pose()
        msg.position.x = (float)(_x)
        msg.position.y = (float)(_y)
        msg.position.z = (float)(_z)
        self.start(msg, frame_id, speed_x, speed_y, speed_z, yaw_mode, yaw_angle, wait)

    # Method simplifications
    def follow_reference(self, _x: float, _y: float, _z: float, frame_id: str, speed_x: float, speed_y: float, speed_z: float) -> None:
        """Follow reference (m) with speed (m/s).

        :type _x: float
        :type _y: float
        :type _z: float
        :type frame_id: str
        :type speed_x: float
        :type speed_y: float
        :type speed_z: float
        """
        self.__follow_reference(_x, _y, _z, frame_id, speed_x, speed_y, speed_z,
                     yaw_mode=YawMode.KEEP_YAW, yaw_angle=None)

    def follow_reference_with_yaw(self, _x: float, _y: float, _z: float, frame_id: str, speed_x: float, speed_y: float, speed_z: float, angle: float) -> None:
        """Follow reference with speed and yaw_angle

        :type _x: float
        :type _y: float
        :type _z: float
        :type frame_id: str
        :type speed_x: float
        :type speed_y: float
        :type speed_z: float
        :type yaw_angle: float
        """
        self.__follow_reference(_x, _y, _z, frame_id, speed_x, speed_y, speed_z,
                     yaw_mode=YawMode.FIXED_YAW, yaw_angle=angle)
        
    def follow_reference_with_reference_facing(self, _x: float, _y: float, _z: float, frame_id: str, speed_x: float, speed_y: float, speed_z: float) -> None:
        """Follow reference with speed and yaw_angle

        :type _x: float
        :type _y: float
        :type _z: float
        :type frame_id: str
        :type speed_x: float
        :type speed_y: float
        :type speed_z: float
        :type yaw_angle: float
        """
        self.__follow_reference(_x, _y, _z, frame_id, speed_x, speed_y, speed_z, yaw_angle=None,
                     yaw_mode=YawMode.YAW_TO_FRAME)

