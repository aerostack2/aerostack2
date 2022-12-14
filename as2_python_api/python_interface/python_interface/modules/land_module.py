"""
land_module.py
"""

import typing

from as2_msgs.action import Land

from python_interface.modules.module_base import ModuleBase
from python_interface.behaviour_actions.land_behaviour import SendLand

if typing.TYPE_CHECKING:
    from ..drone_interface import DroneInterface


class LandModule(ModuleBase):
    """Land Module
    """
    __alias__ = "land"

    def __init__(self, drone: 'DroneInterface') -> None:
        super().__init__(drone, self.__alias__)
        self.__drone = drone

        self.__current_land = None

    def __call__(self, speed: float = 0.5, wait: bool = True) -> None:
        """Land with given speed (m/s).

        :type speed: float
        """
        self.__current_land = SendLand(self.__drone, float(speed))

    def destroy(self):
        """Destroy module, clean exit
        """
        self.__current_land = None

    def pause(self) -> None:
        raise NotImplementedError

    def resume(self) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        if self.__current_land:
            self.__current_land.stop()

    def modify(self, speed) -> None:
        if self.__current_land:
            goal_msg = Land.Goal()
            goal_msg.land_speed = speed
            self.__current_land.modify(goal_msg)
