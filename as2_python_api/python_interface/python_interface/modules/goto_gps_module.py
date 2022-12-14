"""
goto_gps_module.py
"""

from typing import List, TYPE_CHECKING

from as2_msgs.msg import YawMode
from geographic_msgs.msg import GeoPose

from python_interface.modules.module_base import ModuleBase
from python_interface.behaviour_actions.gotowayp_behaviour import SendGoToWaypoint

if TYPE_CHECKING:
    from ..drone_interface import DroneInterface


class GotoGpsModule(ModuleBase):
    """Goto GPS Module
    """
    __alias__ = "goto_gps"

    def __init__(self, drone: 'DroneInterface') -> None:
        super().__init__(drone, self.__alias__)
        self.__drone = drone

        self.__current_goto = None

    def __call__(self, lat: float, lon: float, alt: float, speed: float,
                 yaw_mode: int = YawMode.FIXED_YAW,
                 yaw_angle: float = None, wait: bool = True) -> None:
        """Go to point (m) with speed (m/s).

        :type lat: float
        :type lon: float
        :type alt: float
        :type speed: float
        :type yaw_mode: int
        :type yaw_angle: float
        :type wait: bool
        """
        self.__go_to(lat, lon, alt, speed, yaw_mode, yaw_angle, wait)

    def __go_to(self, lat: float, lon: float, alt: float,
                speed: float, yaw_mode: int, yaw_angle: float, wait: bool = True) -> None:
        msg = GeoPose()
        msg.position.latitude = (float)(lat)
        msg.position.longitude = (float)(lon)
        msg.position.altitude = (float)(alt)

        self.__current_goto = SendGoToWaypoint(
            self.__drone, msg, speed, yaw_mode, yaw_angle, wait)

    # Method simplications
    def go_to_gps(self, lat: float, lon: float, alt: float, speed: float) -> None:
        """Go to GPS point (deg, m) with speed (m/s).

        :type lat: float
        :type lon: float
        :type alt: float
        :type speed: float
        """
        self.__go_to(lat, lon, alt, speed,
                     yaw_mode=YawMode.KEEP_YAW, yaw_angle=None)

    def go_to_gps_with_yaw(self, lat: float, lon: float, alt: float,
                           speed: float, angle: float) -> None:
        """Go to gps position with speed and angle

        :type lat: float
        :type lon: float
        :type alt: float
        :type speed: float
        :type angle: float
        """
        self.__go_to(lat, lon, alt, speed,
                     yaw_mode=YawMode.FIXED_YAW, yaw_angle=angle)

    def go_to_gps_path_facing(self, lat: float, lon: float, alt: float, speed: float) -> None:
        """Go to gps position with speed facing the goal

        :type lat: float
        :type lon: float
        :type alt: float
        :type speed: float
        """
        self.__go_to(lat, lon, alt, speed,
                     yaw_mode=YawMode.PATH_FACING, yaw_angle=None)

    def go_to_gps_point(self, waypoint: List[float], speed: float) -> None:
        """Go to GPS point (deg, m) with speed (m/s).

        :type waypoint: List[float]
        :type speed: float
        """
        self.__go_to(waypoint[0], waypoint[1], waypoint[2],
                     speed, yaw_mode=YawMode.KEEP_YAW, yaw_angle=None)

    def go_to_gps_point_with_yaw(self, waypoint: List[float], speed: float, angle: float) -> None:
        """Go to gps point with speed and yaw angle

        :type waypoint: List[float]
        :type speed: float
        :type angle: float
        """
        self.__go_to(waypoint[0], waypoint[1], waypoint[2],
                     speed, yaw_mode=YawMode.FIXED_YAW, yaw_angle=angle)

    def go_to_gps_point_path_facing(self, waypoint: List[float], speed: float) -> None:
        """Go to gps point with speed facing the goal

        :type waypoint: List[float]
        :type speed: float
        """
        self.__go_to(waypoint[0], waypoint[1], waypoint[2],
                     speed, yaw_mode=YawMode.PATH_FACING, yaw_angle=None)

    def pause(self):
        raise NotImplementedError

    def resume(self):
        raise NotImplementedError

    def stop(self):
        if self.__current_goto:
            self.__current_goto.stop()

    def modify(self, speed):
        raise NotImplementedError

    def destroy(self):
        """Destroy module, clean exit
        """
        self.__current_goto = None
