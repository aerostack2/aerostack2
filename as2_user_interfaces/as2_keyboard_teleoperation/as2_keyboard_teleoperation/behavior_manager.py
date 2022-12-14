"""Behavior manager."""

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

__authors__ = "Javier Melero Deza, Pedro Arias Pérez"
__copyright__ = "Copyright (c) 2022 Universidad Politécnica de Madrid"
__license__ = "BSD-3-Clause"
__version__ = "0.1.0"

from typing import List
from as2_python_api.drone_interface_teleop import DroneInterfaceTeleop as DroneInterface
from as2_keyboard_teleoperation.config_values import AvailableBehaviors


class BehaviorManager:
    """Handle behavior control."""

    def __init__(self, uav_list: List[DroneInterface], drone_id_list):
        self.uav_list = uav_list
        self.drone_id_list = drone_id_list
        self.drone_namespace_list = [
            drone.get_namespace() for drone in uav_list]
        self.drone_dict = dict.fromkeys(
            self.drone_namespace_list, self.uav_list)

    def manage_behavior_control(self, behavior_list, control_order):
        """
        Make de calls to behavior methods.

        :param behavior_list: list of behaviors to be controlled
        :type behavior_list: list(string)
        :param control_order: control_order to be taken uppon behavior list
        :type control_order: string
        """
        for drone_behavior in behavior_list:
            if AvailableBehaviors.BEHAVIOR_TAKE_OFF.value == drone_behavior.split(":")[1]:
                if control_order == "-PAUSE_BEHAVIORS-":
                    self.drone_dict[drone_behavior.split(
                        ":")[0]][0].takeoff.pause()
                elif control_order == "-RESUME_BEHAVIORS-":
                    self.drone_dict[drone_behavior.split(
                        ":")[0]][0].takeoff.resume(False)

            if AvailableBehaviors.BEHAVIOR_LAND.value == drone_behavior.split(":")[1]:
                if control_order == "-PAUSE_BEHAVIORS-":
                    self.drone_dict[drone_behavior.split(
                        ":")[0]][0].land.pause()
                elif control_order == "-RESUME_BEHAVIORS-":
                    self.drone_dict[drone_behavior.split(
                        ":")[0]][0].land.resume(False)

            if AvailableBehaviors.BEHAVIOR_FOLLOW_PATH.value == drone_behavior.split(":")[1]:
                if control_order == "-PAUSE_BEHAVIORS-":
                    self.drone_dict[drone_behavior.split(
                        ":")[0]][0].follow_path.pause()
                elif control_order == "-RESUME_BEHAVIORS-":
                    self.drone_dict[drone_behavior.split(
                        ":")[0]][0].follow_path.resume(False)

            if AvailableBehaviors.BEHAVIOR_GO_TO.value == drone_behavior.split(":")[1]:
                if control_order == "-PAUSE_BEHAVIORS-":
                    self.drone_dict[drone_behavior.split(
                        ":")[0]][0].goto.pause()
                elif control_order == "-RESUME_BEHAVIORS-":
                    self.drone_dict[drone_behavior.split(
                        ":")[0]][0].goto.resume(False)
