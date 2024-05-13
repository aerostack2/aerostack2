"""Follow reference module."""

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


__authors__ = 'Pedro Arias Pérez, Miguel Fernández Cortizas, David Pérez Saura, Rafael Pérez Seguí, \
    Javier Melero Deza'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from typing import TYPE_CHECKING

from as2_msgs.msg import YawMode
from as2_python_api.behavior_actions.follow_reference_behavior import FollowReferenceBehavior
from as2_python_api.modules.module_base import ModuleBase
from geometry_msgs.msg import Pose

if TYPE_CHECKING:
    from ..drone_interface import DroneInterface


class FollowReferenceModule(ModuleBase, FollowReferenceBehavior):
    """Follow reference Module."""

    __alias__ = 'follow_reference'

    def __init__(self, drone: 'DroneInterface') -> None:
        super().__init__(drone, self.__alias__)

    def __call__(
            self, x: float, y: float, z: float, frame_id: str,
            speed_x: float = 0.0, speed_y: float = 0.0, speed_z: float = 0.0,
            yaw_mode: int = YawMode.KEEP_YAW, yaw_angle: float = None, wait: bool = False) -> bool:
        """
        Follow reference.

        :param x: x position relative to the frame_id
        :type x: float
        :param y: y position relative to the frame_id
        :type y: float
        :param z: z position relative to the frame_id
        :type z: float
        :param frame_id: frame_id of the reference
        :type frame_id: str
        :param speed_x: speed limit, defaults to 0.0
        :type speed_x: float, optional
        :param speed_y: speed limit, defaults to 0.0
        :type speed_y: float, optional
        :param speed_z: speed limit, defaults to 0.0
        :type speed_z: float, optional
        :param yaw_mode: yaw mode, defaults to YawMode.KEEP_YAW
        :type yaw_mode: int, optional
        :param yaw_angle: yaw angle (rad) when fixed yaw is set, defaults to None
        :type yaw_angle: float, optional
        :param wait: blocking call, defaults to False
        :type wait: bool, optional
        :return: True if was accepted, False otherwise
        :rtype: bool
        """
        return self.__follow_reference(x, y, z, frame_id, speed_x, speed_y, speed_z,
                                       yaw_mode, yaw_angle, wait)

    def __follow_reference(
            self, x: float, y: float, z: float, frame_id: str,
            speed_x: float, speed_y: float, speed_z: float,
            yaw_mode: int, yaw_angle: float, wait: bool = False) -> bool:
        """
        Follow reference.

        :param x: x position relative to the frame_id
        :type x: float
        :param y: y position relative to the frame_id
        :type y: float
        :param z: z position relative to the frame_id
        :type z: float
        :param frame_id: frame_id of the reference
        :type frame_id: str
        :param speed_x: speed limit
        :type speed_x: float
        :param speed_y: speed limit
        :type speed_y: float
        :param speed_z: speed limit
        :type speed_z: float
        :param yaw_mode: yaw mode
        :type yaw_mode: int
        :param yaw_angle: yaw angle (rad) when fixed yaw is set
        :type yaw_angle: float
        :param wait: blocking call, defaults to False
        :type wait: bool
        :return: True if was accepted, False otherwise
        :rtype: bool
        """
        msg = Pose()
        msg.position.x = (float)(x)
        msg.position.y = (float)(y)
        msg.position.z = (float)(z)
        return self.start(msg, frame_id, speed_x, speed_y,
                          speed_z, yaw_mode, yaw_angle, wait)

    # Method simplifications
    def follow_reference(
            self, x: float, y: float, z: float, frame_id: str,
            speed_x: float, speed_y: float, speed_z: float) -> bool:
        """
        Follow reference. With keep yaw. Non-blocking call.

        :param x: x position relative to the frame_id
        :type x: float
        :param y: y position relative to the frame_id
        :type y: float
        :param z: z position relative to the frame_id
        :type z: float
        :param frame_id: frame_id of the reference
        :type frame_id: str
        :param speed_x: speed limit
        :type speed_x: float
        :param speed_y: speed limit
        :type speed_y: float
        :param speed_z: speed limit
        :type speed_z: float
        """
        return self.__follow_reference(x, y, z, frame_id, speed_x, speed_y, speed_z,
                                       yaw_mode=YawMode.KEEP_YAW, yaw_angle=None)

    def follow_reference_with_yaw(
            self, x: float, y: float, z: float, frame_id: str,
            speed_x: float, speed_y: float, speed_z: float, angle: float) -> bool:
        """
        Follow reference. With desired yaw angle. Non-blocking call.

        :param x: x position relative to the frame_id
        :type x: float
        :param y: y position relative to the frame_id
        :type y: float
        :param z: z position relative to the frame_id
        :type z: float
        :param frame_id: frame_id of the reference
        :type frame_id: str
        :param speed_x: speed limit
        :type speed_x: float
        :param speed_y: speed limit
        :type speed_y: float
        :param speed_z: speed limit
        :type speed_z: float
        :param yaw_angle: yaw angle (rad)
        :type yaw_angle: float
        """
        return self.__follow_reference(x, y, z, frame_id, speed_x, speed_y, speed_z,
                                       yaw_mode=YawMode.FIXED_YAW, yaw_angle=angle)

    def follow_reference_with_reference_facing(
            self, x: float, y: float, z: float, frame_id: str,
            speed_x: float, speed_y: float, speed_z: float) -> bool:
        """
        Follow reference. With reference facing yaw mode. Non-blocking call.

        :param x: x position relative to the frame_id
        :type x: float
        :param y: y position relative to the frame_id
        :type y: float
        :param z: z position relative to the frame_id
        :type z: float
        :param frame_id: frame_id of the reference
        :type frame_id: str
        :param speed_x: speed limit
        :type speed_x: float
        :param speed_y: speed limit
        :type speed_y: float
        :param speed_z: speed limit
        :type speed_z: float
        """
        return self.__follow_reference(x, y, z, frame_id, speed_x, speed_y, speed_z,
                                       yaw_angle=None, yaw_mode=YawMode.YAW_TO_FRAME)
