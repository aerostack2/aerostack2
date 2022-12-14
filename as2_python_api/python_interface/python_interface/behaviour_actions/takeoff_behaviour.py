"""Takeoff action handler"""

# Copyright (c) 2022 Universidad Politécnica de Madrid
# All Rights Reserved
#
# Licensed under the BSD-3-Clause (the "License");
# you may not use this file except in compliance with the License.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


__authors__ = "Miguel Fernández Cortizas, Pedro Arias Pérez, David Pérez Saura, Rafael Pérez Seguí"
__copyright__ = "Copyright (c) 2022 Universidad Politécnica de Madrid"
__license__ = "BSD-3-Clause"
__version__ = "0.1.0"

import typing
from rclpy.action import ActionClient
from as2_msgs.action import TakeOff

from ..behaviour_actions.action_handler import ActionHandler

if typing.TYPE_CHECKING:
    from ..drone_interface_base import DroneInterfaceBase


class SendTakeoff(ActionHandler):
    """Takeoff action"""

    def __init__(self, drone: 'DroneInterfaceBase', height: float, speed: float, wait_result: bool = True) -> None:
        self._action_client = ActionClient(drone, TakeOff, 'TakeOffBehaviour')

        goal_msg = TakeOff.Goal()
        goal_msg.takeoff_height = height
        goal_msg.takeoff_speed = speed

        try:
            super().__init__(self._action_client, goal_msg, drone.get_logger(), wait_result)
        except self.ActionNotAvailable as err:
            drone.get_logger().error(str(err))
        except (self.GoalRejected, self.GoalFailed) as err:
            drone.get_logger().warn(str(err))
