"""RTL Module."""

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

from as2_msgs.msg import YawMode
from as2_python_api.modules.module_base import ModuleBase

if TYPE_CHECKING:
    from ..drone_interface import DroneInterface


class RTLModule(ModuleBase):
    """Return To Launch Module."""

    __alias__ = 'rtl'
    __deps__ = ['go_to', 'land']

    def __init__(self, drone: 'DroneInterface') -> None:
        super().__init__(drone, self.__alias__)
        self.namespace = drone.drone_id

        self.go_to_behavior = getattr(drone, 'go_to')
        self.go_to = getattr(self.go_to_behavior, '__call__')
        self.land_behavior = getattr(drone, 'land')
        self.land = getattr(self.land_behavior, '__call__')
        self.__feedback = None

    @property
    def feedback(self):
        """
        Behavior feedback.

        :return: rclpy.Feedback
        """
        return self.__feedback

    def __call__(self, height: float, speed: float, land_speed: float,
                 yaw_mode: int = YawMode.KEEP_YAW,
                 yaw_angle: float = None, wait: bool = True) -> bool:
        """
        Go to launch point with height (m) and speed (m/s).

        Height frame is map.
        Once reached, land at land_speed (m/s).

        :param height: RTL height (m)
        :type height: float
        :param speed: RTL speed (m/s)
        :type speed: float
        :param land_speed: Land speed (m/s)
        :type land_speed: float
        :param yaw_mode: RTL yaw mode, defaults to YawMode.KEEP_YAW
        :type yaw_mode: int, optional
        :param yaw_angle: RTL yaw angle when fixed yaw is set, defaults to None
        :type yaw_angle: float, optional
        :param wait: blocking call, defaults to True
        :type wait: bool, optional
        :return: True if was accepted, False otherwise
        :rtype: bool
        """
        return self.__rtl(height, speed, land_speed, yaw_mode, yaw_angle, wait)

    def __rtl(self, height: float, speed: float, land_speed: float, yaw_mode: int,
              yaw_angle: float, wait: bool = True) -> None:
        self.__feedback = self.go_to_behavior.feedback
        self.go_to(0.0, 0.0, height, speed, yaw_mode,
                   yaw_angle, frame_id=self.namespace + '/map', wait=True)
        self.__feedback = self.land_behavior.feedback
        self.land(land_speed, wait)

    def destroy(self) -> None:
        """Destroy module, clean exit."""
