"""Point Gimbal Module."""

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


__authors__ = 'Pedro Arias Pérez'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'


from typing import TYPE_CHECKING

from as2_python_api.behavior_actions.point_gimbal_behavior import PointGimbalBehavior
from as2_python_api.modules.module_base import ModuleBase
from geometry_msgs.msg import Pose

if TYPE_CHECKING:
    from ..drone_interface import DroneInterface


class PointGimbalModule(ModuleBase, PointGimbalBehavior):
    """Point Gimbal Module."""

    __alias__ = 'point_gimbal'

    def __init__(self, drone: 'DroneInterface') -> None:
        super().__init__(drone, self.__alias__)

    def __call__(self, x: float, y: float, z: float, frame_id: str, wait: bool = False) -> bool:
        """
        Point Gimbal to reference.

        :param x: x coordinate (m) to point gimbal to
        :type x: float
        :param y: y coordinate (m) to point gimbal to
        :type y: float
        :param z: z coordinate (m) to point gimbal to
        :type z: float
        :param frame_id: reference frame of the coordinates
        :type frame_id: str
        :param wait: blocking call, defaults to True
        :type wait: bool, optional
        :return: True if was accepted, False otherwise
        :rtype: bool
        """
        return self.__point_gimbal(x, y, z, frame_id, wait)

    def __point_gimbal(self, x: float, y: float, z: float,
                       frame_id: str, wait: bool = False) -> None:
        msg = Pose()
        msg.position.x = (float)(x)
        msg.position.y = (float)(y)
        msg.position.z = (float)(z)
        self.start(msg, frame_id, wait)

    # Method simplifications
    def point_gimbal(self, x: float, y: float, z: float, frame_id: str) -> bool:
        """
        Point Gimbal to reference, blocking call.

        :param x: x coordinate (m) to point gimbal to
        :type x: float
        :param y: y coordinate (m) to point gimbal to
        :type y: float
        :param z: z coordinate (m) to point gimbal to
        :type z: float
        :param frame_id: reference frame of the coordinates
        :type frame_id: str
        :return: True if was accepted, False otherwise
        :rtype: bool
        """
        return self.__point_gimbal(x, y, z, frame_id)
