"""
takeoff_module.py
"""

import typing

from as2_msgs.action import TakeOff

from python_interface.modules.module_base import ModuleBase
from python_interface.behaviour_actions.takeoff_behaviour import SendTakeoff

if typing.TYPE_CHECKING:
    from ..drone_interface import DroneInterface


class TakeoffModule(ModuleBase):
    """Takeoff Module
    """
    __alias__ = "takeoff"

    def __init__(self, drone: 'DroneInterface') -> None:
        super().__init__(drone, self.__alias__)
        self.__drone = drone

        self.__current_tk = None

    def __call__(self, height: float = 1.0, speed: float = 0.5, wait: bool = True) -> None:
        """Takeoff to given height (m) and given speed (m/s).

        :type height: float
        :type speed: float
        """
        self.__current_tk = SendTakeoff(
            self.__drone, float(height), float(speed))

    def destroy(self):
        """Destroy module, clean exit
        """
        self.__current_tk = None

    def pause(self) -> None:
        raise NotImplementedError

    def resume(self) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        if self.__current_tk:
            self.__current_tk.stop()

    def modify(self, height, speed) -> None:
        if self.__current_tk:
            goal_msg = TakeOff.Goal()
            goal_msg.takeoff_height = height
            goal_msg.takeoff_speed = speed
        raise NotImplementedError
