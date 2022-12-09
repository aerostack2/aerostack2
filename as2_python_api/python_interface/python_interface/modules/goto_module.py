from typing import List

from as2_msgs.msg import YawMode
from geometry_msgs.msg import Pose
from geographic_msgs.msg import GeoPose

from python_interface.behaviour_actions.gotowayp_behaviour import SendGoToWaypoint


class GotoModule:
    __alias__ = "goto"

    def __init__(self, drone) -> None:
        self.__drone = drone
        self.__drone.modules[self.__alias__] = self

    # def __call__(self, speed: float = 0.5) -> None:
    #     """Drone landing"""
    #     SendGoToWaypoint(self.__drone, float(speed))

    def __go_to(self, _x: float, _y: float, _z: float,
                speed: float, yaw_mode: int, yaw_angle: float, is_gps: bool) -> None:
        if is_gps:
            msg = GeoPose()
            msg.position.latitude = (float)(_x)
            msg.position.longitude = (float)(_y)
            msg.position.altitude = (float)(_z)
        else:
            msg = Pose()
            msg.position.x = (float)(_x)
            msg.position.y = (float)(_y)
            msg.position.z = (float)(_z)

        SendGoToWaypoint(self, msg, speed, yaw_mode, yaw_angle)

    def go_to(self, _x: float, _y: float, _z: float, speed: float) -> None:
        """Go to point (m) with speed (m/s).

        :type _x: float
        :type _y: float
        :type _z: float
        :type speed: float
        """
        self.__go_to(_x, _y, _z, speed, yaw_mode=YawMode.KEEP_YAW,
                     yaw_angle=None, is_gps=False)

    def go_to_with_yaw(self, _x: float, _y: float, _z: float, speed: float, angle: float) -> None:
        """Go to position with speed and yaw_angle

        :type _x: float
        :type _y: float
        :type _z: float
        :type speed: float
        :type yaw_angle: float
        """
        self.__go_to(_x, _y, _z, speed, yaw_mode=YawMode.FIXED_YAW,
                     yaw_angle=angle, is_gps=False)

    def go_to_path_facing(self, _x: float, _y: float, _z: float, speed: float) -> None:
        """Go to position facing goal with speed

        :type _x: float
        :type _y: float
        :type _z: float
        :type speed: float
        """
        self.__go_to(_x, _y, _z, speed, yaw_mode=YawMode.PATH_FACING,
                     yaw_angle=None, is_gps=False)

    def go_to_point(self, point: List[float], speed: float) -> None:
        """Go to point (m) with speed (m/s).

        :type point: List[float]
        :type speed: float
        """
        self.__go_to(point[0], point[1], point[2],
                     speed, yaw_mode=YawMode.KEEP_YAW, yaw_angle=None, is_gps=False)

    def go_to_point_with_yaw(self, point: List[float], speed: float, angle: float) -> None:
        """Go to point with speed and yaw_angle

        :type point: List[float]
        :type speed: float
        :type ignore_yaw: bool, optional
        """
        self.__go_to(point[0], point[1], point[2],
                     speed, yaw_mode=YawMode.FIXED_YAW, yaw_angle=angle, is_gps=False)

    def go_to_point_path_facing(self, point: List[float], speed: float) -> None:
        """Go to point facing goal with speed

        :type point: List[float]
        :type speed: float
        """
        self.__go_to(point[0], point[1], point[2],
                     speed, yaw_mode=YawMode.PATH_FACING, yaw_angle=None, is_gps=False)

    def go_to_gps(self, lat: float, lon: float, alt: float, speed: float) -> None:
        """Go to GPS point (deg, m) with speed (m/s).

        :type lat: float
        :type lon: float
        :type alt: float
        :type speed: float
        """
        self.__go_to(lat, lon, alt, speed, yaw_mode=YawMode.KEEP_YAW,
                     yaw_angle=None, is_gps=True)

    def go_to_gps_with_yaw(self, lat: float, lon: float, alt: float, speed: float, angle: float) -> None:
        """Go to gps position with speed and angle

        :type lat: float
        :type lon: float
        :type alt: float
        :type speed: float
        :type angle: float
        """
        self.__go_to(lat, lon, alt, speed, yaw_mode=YawMode.FIXED_YAW,
                     yaw_angle=angle, is_gps=True)

    def go_to_gps_path_facing(self, lat: float, lon: float, alt: float, speed: float) -> None:
        """Go to gps position with speed facing the goal

        :type lat: float
        :type lon: float
        :type alt: float
        :type speed: float
        """
        self.__go_to(lat, lon, alt, speed, yaw_mode=YawMode.PATH_FACING,
                     yaw_angle=None, is_gps=True)

    def go_to_gps_point(self, waypoint: List[float], speed: float) -> None:
        """Go to GPS point (deg, m) with speed (m/s).

        :type waypoint: List[float]
        :type speed: float
        """
        self.__go_to(waypoint[0], waypoint[1], waypoint[2],
                     speed, yaw_mode=YawMode.KEEP_YAW, yaw_angle=None, is_gps=True)

    def go_to_gps_point_with_yaw(self, waypoint: List[float], speed: float, angle: float) -> None:
        """Go to gps point with speed and yaw angle

        :type waypoint: List[float]
        :type speed: float
        :type angle: float
        """
        self.__go_to(waypoint[0], waypoint[1], waypoint[2],
                     speed, yaw_mode=YawMode.FIXED_YAW, yaw_angle=angle, is_gps=True)

    def go_to_gps_point_path_facing(self, waypoint: List[float], speed: float) -> None:
        """Go to gps point with speed facing the goal

        :type waypoint: List[float]
        :type speed: float
        """
        self.__go_to(waypoint[0], waypoint[1], waypoint[2],
                     speed, yaw_mode=YawMode.PATH_FACING, yaw_angle=None, is_gps=True)

    # TODO
    def __del__(self):
        del self.__drone.modules[self.__alias__]

    def pause(self):
        raise NotImplementedError

    def resume(self):
        raise NotImplementedError

    def stop(self):
        raise NotImplementedError

    def modify(self, speed):
        raise NotImplementedError
