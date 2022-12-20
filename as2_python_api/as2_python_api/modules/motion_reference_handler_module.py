"""
motion_reference_handler_module.py
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


__authors__ = "Pedro Arias Pérez, Miguel Fernández Cortizas, David Pérez Saura, Rafael Pérez Seguí"
__copyright__ = "Copyright (c) 2022 Universidad Politécnica de Madrid"
__license__ = "BSD-3-Clause"
__version__ = "0.1.0"

import typing

from std_srvs.srv import SetBool

from as2_python_api.modules.module_base import ModuleBase
from as2_motion_reference_handlers.hover_motion import HoverMotion
from as2_motion_reference_handlers.position_motion import PositionMotion
from as2_motion_reference_handlers.speed_motion import SpeedMotion
from as2_motion_reference_handlers.speed_in_a_plane import SpeedInAPlaneMotion

if typing.TYPE_CHECKING:
    from ..drone_interface import DroneInterface


class MotionReferenceHandlerModule(ModuleBase):
    """Motion Reference Handlers module"""
    __alias__ = "motion_ref_handler"

    def __init__(self, drone: 'DroneInterface') -> None:
        super().__init__(drone, self.__alias__)
        self.__drone = drone

        # FIXME: temporaly, manually stoping behaviors
        self.trajectory_gen_cli = self.__drone.create_client(
            SetBool, "traj_gen/run_node")
        if not self.trajectory_gen_cli.wait_for_service(timeout_sec=3):
            self.__drone.get_logger().warn("Trajectory generator service not found")
            self.trajectory_gen_cli = None

        self.__hover_motion_handler = HoverMotion(self.__drone)
        self.position = PositionMotion(self.__drone)
        self.speed = SpeedMotion(self.__drone)
        self.speed_in_a_plane = SpeedInAPlaneMotion(self.__drone)

    def hover(self) -> None:
        """Stop and hover current position.
        """
        if self.trajectory_gen_cli is not None:
            self.__drone.get_logger().info("Calling trajectory generator")
            req = SetBool.Request()
            req.data = False
            resp = self.trajectory_gen_cli.call(req)
            if not resp.success:
                self.__drone.get_logger().warn("Cannot stop trajectory generator")
        self.__hover_motion_handler.send_hover()
        self.__drone.get_logger().info("Hover sent")

    def destroy(self) -> None:
        """Destroy module, clean exit
        """
        self.__drone.destroy_client(self.trajectory_gen_cli)

        self.__hover_motion_handler = None
        self.position = None
        self.speed = None
        self.speed_in_a_plane = None
