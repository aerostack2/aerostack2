"""
follow_path_gps_module.py
"""

import typing

from as2_msgs.msg import YawMode
from nav_msgs.msg import Path
from geographic_msgs.msg import GeoPath

from python_interface.modules.module_base import ModuleBase
from python_interface.behaviour_actions.followpath_behaviour import SendFollowPath

if typing.TYPE_CHECKING:
    from ..drone_interface import DroneInterface


class FollowPathGpsModule(ModuleBase):
    """Follow Path GPS Module
    """
    __alias__ = "follow_path_gps"

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
        # Convert path to GeoPath
        gps_path = GeoPath()
        # TODO: Convert header
        # gps_path.header
        for pose in path.poses:
            gps_path.poses.append(pose.pose)

        self.__current_fp = SendFollowPath(
            self.__drone, gps_path, speed, yaw_mode, yaw_angle, wait)

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

    def modify(self, wp_path: Path, speed: float,
               yaw_mode: int = YawMode.KEEP_YAW) -> None:
        # path_data = SendFollowPath.FollowPathData(wp_path, speed, yaw_mode, is_gps=True)
        # # path_data to goal_msg
        # self.__current_fp.modify(goal_msg=msg)
        raise NotImplementedError

    def destroy(self):
        """Destroy module, clean exit
        """
        self.__current_fp = None
