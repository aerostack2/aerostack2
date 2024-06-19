"""Behavior manager."""

from __future__ import annotations

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

__authors__ = 'Javier Melero Deza, Pedro Arias Pérez, Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from threading import Thread, ThreadError
from typing import Union

from as2_python_api.behavior_actions.behavior_handler import BehaviorHandler
from as2_python_api.drone_interface_base import DroneInterfaceBase


class ThreadWithReturnValue(Thread):

    def __init__(self, group=None, target=None, name=None,
                 args=(), kwargs={}, Verbose=None):
        Thread.__init__(self, group, target, name, args, kwargs)
        self._return = None

    def run(self):
        if self._target is not None:
            self._return = self._target(*self._args,
                                        **self._kwargs)

    def join(self, *args):
        Thread.join(self, *args)
        return self._return


class DroneBehaviorManager:
    """Handle behavior control."""

    @staticmethod
    def pause_behaviors(behaviors: Union[list, str], uav: DroneInterfaceBase):
        """
        Pause all behaviors in list.

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
        """
        Resume all behaviors in list.

        :param behavior: _description_
        :type behavior: _type_
        :return: _description_
        :rtype: bool
        """
        return DroneBehaviorManager.drone_behavior_func(behaviors, uav, 'resume')

    @staticmethod
    def stop_behaviors(behaviors: Union[list, str], uav: DroneInterfaceBase):
        """
        Stop all behaviors in list.

        :param behavior: _description_
        :type behavior: _type_
        :return: _description_
        :rtype: bool
        """
        return DroneBehaviorManager.drone_behavior_func(behaviors, uav, 'stop')

    @staticmethod
    def pause_all_behaviors(uav: DroneInterfaceBase):
        """
        Pause all behaviors for a drone.

        :return: _description_
        :rtype: dict {behavior: bool}
        """
        success = {behavior: uav.modules[behavior].pause()
                   for behavior in uav.modules if isinstance(uav.modules[behavior],
                                                             BehaviorHandler)}
        return success

    @staticmethod
    def resume_all_behaviors(uav: DroneInterfaceBase):
        """
        Resume all behaviors for a drone.

        :return: _description_
        :rtype: dict {behavior: bool}
        """
        success = {behavior: uav.modules[behavior].resume(False)
                   for behavior in uav.modules if isinstance(uav.modules[behavior],
                                                             BehaviorHandler)}

        return success

    @staticmethod
    def stop_all_behaviors(uav: DroneInterfaceBase):
        """
        Stop all behaviors for a drone.

        :return: _description_
        :rtype: dict {behavior: bool}
        """
        success = {behavior: uav.modules[behavior].stop()
                   for behavior in uav.modules if isinstance(uav.modules[behavior],
                                                             BehaviorHandler)}
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
    def drone_behavior_func(behaviors: Union[list, str], uav: DroneInterfaceBase, func):
        """
        Call behavior method.

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
                success[behavior] = getattr(uav.modules[behavior], func)(
                    False) if func == 'resume' else getattr(uav.modules[behavior], func)()

            except KeyError:
                uav.get_logger().error(f'{behavior} not found.')
            except AttributeError as at_ex:
                uav.get_logger().error(
                    f'{behavior} is not a behavior: {at_ex}')

        return success


class SwarmBehaviorManager:
    """Swam Behavior Manager."""

    @staticmethod
    def pause_behaviors(behavior_dict):
        """
        Pause behaviors.

        :param behavior_dict: _description_
        :type behavior_dict: _type_
        :return: _description_
        :rtype: dict {drone_id:{behavior: bool}}
        """
        return SwarmBehaviorManager.swarm_behavior_func(behavior_dict, 'pause_behaviors')

    @staticmethod
    def resume_behaviors(behavior_dict):
        """
        Resume behaviors.

        :param behavior_dict: _description_
        :type behavior_dict: _type_
        :return: _description_
        :rtype: dict {drone_id:{behavior: bool}}
        """
        return SwarmBehaviorManager.swarm_behavior_func(behavior_dict, 'resume_behaviors')

    @staticmethod
    def stop_behaviors(behavior_dict):
        """
        Stop behaviors.

        :param behavior_dict: _description_
        :type behavior_dict: _type_
        :return: _description_
        :rtype: dict {drone_id:{behavior: bool}}
        """
        return SwarmBehaviorManager.swarm_behavior_func(behavior_dict, 'stop_behaviors')

    @staticmethod
    def pause_all_behaviors(drone_interface_list: list[DroneInterfaceBase]):
        """
        Pause all behaviors for each interface.

        :param drone_id_list: _description_, defaults to None
        :type drone_id_list: _type_, optional
        :return: _description_
        :rtype: dict {drone_id:{behavior: bool}}
        """
        return SwarmBehaviorManager.swarm_all_behavior_func(drone_interface_list,
                                                            'pause_all_behaviors')

    @staticmethod
    def resume_all_behaviors(drone_interface_list: list[DroneInterfaceBase]):
        """
        Resume all behaviors for all drones in the swarm.

        :param drone_id_list: _description_, defaults to None
        :type drone_id_list: _type_, optional
        :return: _description_
        :rtype: dict {drone_id:{behavior: bool}}
        """
        return SwarmBehaviorManager.swarm_all_behavior_func(drone_interface_list,
                                                            'resume_all_behaviors')

    @staticmethod
    def stop_all_behaviors(drone_interface_list: list[DroneInterfaceBase]):
        """
        Stop all behaviors for all drones in the swarm.

        :param drone_id_list: _description_, defaults to None
        :type drone_id_list: _type_, optional
        :return: _description_
        :rtype: dict {drone_id:{behavior: bool}}
        """
        return SwarmBehaviorManager.swarm_all_behavior_func(drone_interface_list,
                                                            'stop_all_behaviors')

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
    def swarm_behavior_func(behavior_dict, func):
        """
        Execute a behavior function for all drones in the swarm.

        :param func: _description_
        :type func: _type_
        :param behavior_dict: _description_
        :type behavior_dict: _type_
        :return: _description_
        :rtype: dict {drone_id:{behavior: bool}}
        """
        threads = {}
        success = {}
        for drone_interface in behavior_dict:
            try:
                _t = ThreadWithReturnValue(target=getattr(DroneBehaviorManager, func), args=(
                    behavior_dict[drone_interface], drone_interface,))
                threads[drone_interface.drone_id] = _t
                _t.start()
            except ThreadError as _e:
                drone_interface.get_logger().error(f'{_e}')

        for drone_id in threads:
            success[drone_id] = threads[drone_id].join()

        return success

    @staticmethod
    def swarm_all_behavior_func(drone_interface_list, func):
        """
        Execute a behavior function for all drones in the swarm.

        :param func: _description_
        :type func: _type_
        :param drone_interface_list: _description_
        :type drone_interface_list: _type_
        :return: _description_
        :rtype: dict {drone_id:{behavior: bool}}
        """
        threads = {}
        success = {}
        for drone_interface in drone_interface_list:
            try:
                _t = ThreadWithReturnValue(target=getattr(DroneBehaviorManager, func), args=(
                    drone_interface,))
                threads[drone_interface.drone_id] = _t
                _t.start()
            except ThreadError as _e:
                drone_interface.get_logger().error(f'{_e}')

        for drone_id in threads:
            success[drone_id] = threads[drone_id].join()

        return success
