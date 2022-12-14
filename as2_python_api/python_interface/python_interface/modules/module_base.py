"""
module_base.py
"""

from typing import TYPE_CHECKING
import abc
if TYPE_CHECKING:
    from ..drone_interface import DroneInterface


class ModuleBase(abc.ABC):
    """Module Base
    """
    __alias__ = ""

    def __init__(self, drone: 'DroneInterface', alias: str) -> None:
        self.__drone = drone
        self.__alias__ = alias
        self.__drone.modules[self.__alias__] = self

    def __del__(self):
        try:
            # Delete when unloading module
            del self.__drone.modules[self.__alias__]
        except KeyError:
            pass  # Avoid exception when DroneInterface destruction

    @abc.abstractmethod
    def destroy(self):
        """Self destroy, must override

        :raises NotImplementedError: if not overriden
        """
        raise NotImplementedError
