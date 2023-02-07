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

from typing import Union
from as2_python_api.behavior_actions.behavior_handler import BehaviorHandler
from as2_python_api.drone_interface_base import DroneInterfaceBase


class DroneBehaviorManager:
    """Handle behavior control."""

    @staticmethod
    def pause_behaviors(behaviors: Union[list, str], uav: DroneInterfaceBase):
        """_summary_

        :param behavior: _description_
        :type behavior: _type_
        :param uav: _description_
        :type uav: DroneInterfaceBase
        :return: _description_
        :rtype: _type_
        """
        return DroneBehaviorManager.drone_behavior_func(behaviors, uav, 'pause')

    @staticmethod
    def resume_behaviors(behaviors: Union[list, str], uav: DroneInterfaceBase):
        """_summary_

        :param behavior: _description_
        :type behavior: _type_
        :return: _description_
        :rtype: bool
        """
        return DroneBehaviorManager.drone_behavior_func(behaviors, uav, 'resume')

    @staticmethod
    def stop_behaviors(behaviors: list | str, uav: DroneInterfaceBase):
        """_summary_

        :param behavior: _description_
        :type behavior: _type_
        :return: _description_
        :rtype: bool
        """

        return DroneBehaviorManager.drone_behavior_func(behaviors, uav, 'stop')

    @staticmethod
    def pause_all_behaviors(uav: DroneInterfaceBase):
        """_summary_

        :return: _description_
        :rtype: dict {behavior: bool}
        """
        success = {behavior: uav.modules[behavior].pause()
                   for behavior in uav.modules if isinstance(uav.modules[behavior], BehaviorHandler)}
        return success

    @staticmethod
    def resume_all_behaviors(uav: DroneInterfaceBase):
        """_summary_

        :return: _description_
        :rtype: dict {behavior: bool}
        """
        success = {behavior: uav.modules[behavior].resume()
                   for behavior in uav.modules if isinstance(uav.modules[behavior], BehaviorHandler)}
        return success

    @staticmethod
    def stop_all_behaviors(uav: DroneInterfaceBase):
        """_summary_

        :return: _description_
        :rtype: dict {behavior: bool}
        """
        success = {behavior: uav.modules[behavior].stop()
                   for behavior in uav.modules if isinstance(uav.modules[behavior], BehaviorHandler)}
        return success

    @staticmethod
    def get_behavior_status(uav: DroneInterfaceBase):
        """
        Get behavior status for an interface.

        :return: dictionary with namespace and behavior status
        :rtype: dict(namespace, list(int))
        """
        status = {key: uav.modules[key].status for key in uav.modules
                  if isinstance(uav.modules[key], BehaviorHandler)}
        return status

    @staticmethod
    def drone_behavior_func(behaviors: list | str, uav: DroneInterfaceBase, func):
        """_summary_

        :param behavior: _description_
        :type behavior: list | str
        :param uav: _description_
        :type uav: DroneInterfaceBase
        :param func: _description_
        :type func: _type_
        :return: _description_
        :rtype: _type_
        """
        success = {}

        if not isinstance(behaviors, list):
            behaviors = [behaviors]

        for behavior in behaviors:
            try:
                success[behavior] = getattr(uav.modules[behavior], func)()
            except KeyError:
                uav.get_logger().error(f'{behavior} not found.')
            except AttributeError:
                uav.get_logger().error(f'{behavior} is not a behavior.')

        return success


class SwarmBehaviorManager:
    """_summary_"""

    @staticmethod
    def pause_behaviors(behavior_dict):
        """_summary_

        :param behavior_dict: _description_
        :type behavior_dict: _type_
        :return: _description_
        :rtype: dict {drone_id:{behavior: bool}}
        """
        return SwarmBehaviorManager.swarm_behavior_func('pause_behaviors', behavior_dict)

    @staticmethod
    def resume_behaviors(behavior_dict):
        """_summary_

        :param behavior_dict: _description_
        :type behavior_dict: _type_
        :return: _description_
        :rtype: dict {drone_id:{behavior: bool}}
        """
        return SwarmBehaviorManager.swarm_behavior_func('resume_behaviors', behavior_dict)

    @staticmethod
    def stop_behaviors(behavior_dict):
        """_summary_

        :param behavior_dict: _description_
        :type behavior_dict: _type_
        :return: _description_
        :rtype: dict {drone_id:{behavior: bool}}
        """
        return SwarmBehaviorManager.swarm_behavior_func('stop_behaviors', behavior_dict)

    @staticmethod
    def pause_all_behaviors(drone_interface_list: list[DroneInterfaceBase]):
        """_summary_

        :param drone_id_list: _description_, defaults to None
        :type drone_id_list: _type_, optional
        :return: _description_
        :rtype: dict {drone_id:{behavior: bool}}
        """
        success = {drone_interface.drone_id: DroneBehaviorManager.pause_all_behaviors(
            drone_interface) for drone_interface in drone_interface_list}
        return success

    @staticmethod
    def resume_all_behaviors(drone_interface_list: list[DroneInterfaceBase]):
        """_summary_

        :param drone_id_list: _description_, defaults to None
        :type drone_id_list: _type_, optional
        :return: _description_
        :rtype: dict {drone_id:{behavior: bool}}
        """
        success = {drone_interface.drone_id: DroneBehaviorManager.resume_all_behaviors(
            drone_interface) for drone_interface in drone_interface_list}
        return success

    @staticmethod
    def stop_all_behaviors(drone_interface_list: list[DroneInterfaceBase]):
        """_summary_

        :param drone_id_list: _description_, defaults to None
        :type drone_id_list: _type_, optional
        :return: _description_
        :rtype: dict {drone_id:{behavior: bool}}
        """
        success = {drone_interface.drone_id: DroneBehaviorManager.stop_all_behaviors(
            drone_interface) for drone_interface in drone_interface_list}
        return success

    @staticmethod
    def get_behaviors_status(drone_interface_list: list[DroneInterfaceBase]):
        """
        Get behavior status for each interface.

        :return: dictionary with namespace and behavior status
        :rtype: dict {drone_id:{behavior:status(int)}}
        """
        status = {drone_interface.drone_id: DroneBehaviorManager.get_behavior_status(
            drone_interface) for drone_interface in drone_interface_list}

        return status

    @staticmethod
    def swarm_behavior_func(func, behavior_dict):
        """_summary_

        :param func: _description_
        :type func: _type_
        :param behavior_dict: _description_
        :type behavior_dict: _type_
        :return: _description_
        :rtype: dict {drone_id:{behavior: bool}}
        """
        success = {drone_interface.drone_id: getattr(DroneBehaviorManager, func)(
            behavior_dict[drone_interface], drone_interface) for drone_interface in behavior_dict}
        return success
