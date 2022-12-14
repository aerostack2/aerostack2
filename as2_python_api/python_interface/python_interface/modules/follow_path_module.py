"""
follow_path_module.py
"""

import typing

from as2_msgs.msg import YawMode
from nav_msgs.msg import Path

from python_interface.modules.module_base import ModuleBase
from python_interface.behaviour_actions.followpath_behaviour import SendFollowPath

if typing.TYPE_CHECKING:
    from ..drone_interface import DroneInterface


class FollowPathModule(ModuleBase):
    """Follow Path Module
    """
    __alias__ = "follow_path"

    def __init__(self, drone: 'DroneInterface') -> None:
        super().__init__(drone, self.__alias__)
        self.__drone = drone

        self.__current_fp = None

    def __call__(self, path: Path, speed: float,
                 yaw_mode: int = YawMode.KEEP_YAW,
                 yaw_angle: float = None, wait: bool = True) -> None:
        """Follow path with speed (m/s) and yaw_mode.

        :type path: Path
        :type speed: float
        :type yaw_mode: int
        :type yaw_angle: float
        :type wait: bool
        """
        self.__follow_path(path, speed, yaw_mode, yaw_angle, wait)

    def __follow_path(self, path: Path,
                      speed: float, yaw_mode: int, yaw_angle: float, wait: bool = True) -> None:
        self.__current_fp = SendFollowPath(
            self.__drone, path, speed, yaw_mode, yaw_angle, wait)

    # Method simplifications
    def follow_path_with_yaw(self, path: Path, speed: float) -> None:
        """Follow path with speed and keep yaw
        
        :type path: Path
        :type speed: float
        """
        self.__follow_path(path, speed,
                           yaw_mode=YawMode.KEEP_YAW, yaw_angle=0.0)
        
    def follow_path_with_yaw(self, path: Path, speed: float, angle: float) -> None:
        """Follow path with speed and yaw_angle

        :type path: Path
        :type speed: float
        :type yaw_angle: float
        """
        self.__follow_path(path, speed,
                           yaw_mode=YawMode.FIXED_YAW, yaw_angle=angle)

    def follow_path_with_path_facing(self, path: Path, speed: float) -> None:
        """Follow path with path facing and with speed
        
        :type path: Path
        :type speed: float
        """
        self.__follow_path(path, speed,
                           yaw_mode=YawMode.PATH_FACING, yaw_angle=0.0)

    def pause(self) -> None:
        # self.__current_fp.pause()
        # super().pause()  # Best way to do it. Take advantage of inheritance or multi-inheritance
        raise NotImplementedError

    def resume(self) -> None:
        # self.__current_fp.resume()
        raise NotImplementedError

    def stop(self) -> None:
        if self.__current_fp:
            self.__current_fp.stop()

    def modify(self, path: Path, speed: float,
               yaw_mode: int = YawMode.KEEP_YAW) -> None:
        # path_data = SendFollowPath.FollowPathData(path, speed, yaw_mode, is_gps=False)
        # # path_data to goal_msg
        # self.__current_fp.modify(goal_msg=msg)
        raise NotImplementedError

    def destroy(self):
        """Destroy module, clean exit
        """
        self.__current_fp = None
