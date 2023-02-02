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

__authors__ = "Javier Melero Deza, Pedro Arias Pérez, Rafael Pérez Seguí"
__copyright__ = "Copyright (c) 2022 Universidad Politécnica de Madrid"
__license__ = "BSD-3-Clause"
__version__ = "0.1.0"

from typing import List
from enum import Enum
from as2_python_api.drone_interface_teleop import DroneInterfaceTeleop as DroneInterface


class ExtendedEnum(Enum):

    @classmethod
    def list(cls):
        return list(map(lambda c: c.value, cls))


class AvailableBehaviors(ExtendedEnum):
    BEHAVIOR_TAKE_OFF = "Behavior Take Off"
    BEHAVIOR_LAND = "Behavior Land"
    BEHAVIOR_FOLLOW_PATH = "Behavior Follow Path"
    BEHAVIOR_GO_TO = "Behavior Go To"


class DroneBehaviorManager:
    """Handle behavior control."""

    def __init__(self, uav: DroneInterface):
        self.uav = uav
        self.behavior_dict = {AvailableBehaviors.BEHAVIOR_TAKE_OFF.value: self.uav.takeoff,
                              AvailableBehaviors.BEHAVIOR_LAND.value: self.uav.land,
                              AvailableBehaviors.BEHAVIOR_FOLLOW_PATH.value: self.uav.follow_path,
                              AvailableBehaviors.BEHAVIOR_GO_TO.value: self.uav.goto}

    def pause_behavior(self, behavior):
        """_summary_

        :param behavior: _description_
        :type behavior: _type_
        :return: _description_
        :rtype: _type_
        """
        return self.behavior_dict[behavior].pause()

    def resume_behavior(self, behavior):
        """_summary_

        :param behavior: _description_
        :type behavior: _type_
        :return: _description_
        :rtype: _type_
        """
        return self.behavior_dict[behavior].resume(False)

    def get_behavior_status(self):
        """
        Get behavior status for each interface.

        :return: dictionary with namespace and behavior status
        :rtype: dict(namespace, list(int))
        """
        dct = {
            key: self.behavior_dict[key].status for key in self.behavior_dict}
        self.uav.get_logger().info(str(dct))
        return dct


class SwarmBehaviorManager:
    """_summary_"""

    def __init__(self, uav_list: List[DroneInterface]):
        self.behavior_manager_list = []
        self.uav_list = uav_list
        for uav in uav_list:
            self.behavior_manager_list.append(DroneBehaviorManager(uav=uav))

    def pause_movements_behaviors(self, behavior_dict):
        """_summary_

        :param behavior_dict: _description_
        :type behavior_dict: _type_
        """
        success = {}
        for behavior_manager in self.behavior_manager_list:
            if behavior_manager.uav.get_namespace() in behavior_dict:
                for behavior in behavior_dict[behavior_manager.uav.get_namespace()]:

                    success_beh = {key: behavior_manager.pause_behavior(
                        behavior) for key in behavior_dict[behavior_manager.uav.get_namespace()]}
                    success[behavior_manager.uav.get_namespace()] = success_beh
        return success

    def resume_movements_behaviors(self, behavior_dict):
        """_summary_

        :param behavior_dict: _description_
        :type behavior_dict: _type_
        """
        success = {}
        for behavior_manager in self.behavior_manager_list:
            if behavior_manager.uav.get_namespace() in behavior_dict:
                for behavior in behavior_dict[behavior_manager.uav.get_namespace()]:
                    success_beh = {key: behavior_manager.resume_behavior(
                        behavior) for key in behavior_dict[behavior_manager.uav.get_namespace()]}
                    success[behavior_manager.uav.get_namespace()] = success_beh
        return success

    def get_behaviors_status(self):
        """
        Get behavior status for each interface.

        :return: dictionary with namespace and behavior status
        :rtype: dict(namespace, list(int))
        """
        status_dict = {}
        for behavior_manager in self.behavior_manager_list:

            status_dict[behavior_manager.uav.get_namespace(
            )] = behavior_manager.get_behavior_status()

        return status_dict
