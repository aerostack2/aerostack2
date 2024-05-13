"""Takeoff Behavior."""

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


__authors__ = 'Miguel Fernández Cortizas, Pedro Arias Pérez, David Pérez Saura, Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import typing

from as2_msgs.action import Takeoff

from ..behavior_actions.behavior_handler import BehaviorHandler

if typing.TYPE_CHECKING:
    from ..drone_interface_base import DroneInterfaceBase


class TakeoffBehavior(BehaviorHandler):
    """Takeoff Behavior."""

    def __init__(self, drone: 'DroneInterfaceBase') -> None:
        self.__drone = drone

        try:
            super().__init__(drone, Takeoff, 'TakeoffBehavior')
        except self.BehaviorNotAvailable as err:
            self.__drone.get_logger().warn(str(err))

    def start(self, height: float, speed: float, wait_result: bool = True) -> bool:
        """Start behavior."""
        goal_msg = Takeoff.Goal()
        goal_msg.takeoff_height = float(height)
        goal_msg.takeoff_speed = float(speed)

        try:
            return super().start(goal_msg, wait_result)
        except self.GoalRejected as err:
            self.__drone.get_logger().warn(str(err))
        return False

    def modify(self, height: float, speed: float) -> bool:
        """Modify behavior."""
        goal_msg = Takeoff.Goal()
        goal_msg.takeoff_height = height
        goal_msg.takeoff_speed = speed

        return super().modify(goal_msg)
