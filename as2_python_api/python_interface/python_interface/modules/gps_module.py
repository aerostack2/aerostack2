"""
gps_module.py
"""
from typing import List

from rclpy.qos import qos_profile_sensor_data
from as2_msgs.srv import SetOrigin, GeopathToPath, PathToGeopath
from sensor_msgs.msg import NavSatFix

from python_interface.shared_data.gps_data import GpsData


class GpsModule:
    """GPS module"""
    __alias__ = "gps"

    def __init__(self, drone) -> None:
        self.__drone = drone
        self.__drone.modules[self.__alias__] = self

        self.__drone.use_gps = True
        self.gps = GpsData()
        self.set_origin_cli_ = self.__drone.create_client(
            SetOrigin, "set_origin")
        if not self.set_origin_cli_.wait_for_service(timeout_sec=3):
            self.__drone.get_logger().warn("Set Origin not ready")

        self.gps_sub = self.__drone.create_subscription(
            NavSatFix, 'sensor_measurements/gps', self.__gps_callback, qos_profile_sensor_data)

        # self.global_to_local_cli_ = self.create_client(
        #     GeopathToPath, f"{translator_namespace}/geopath_to_path")
        # self.local_to_global_cli_ = self.create_client(
        #     PathToGeopath, f"{translator_namespace}/path_to_geopath")

    def __gps_callback(self, msg: NavSatFix) -> None:
        """navdata (gps) callback"""
        self.gps.fix = [msg.latitude, msg.longitude, msg.altitude]

    @property
    def gps_pose(self) -> List[float]:
        """gps pose getter"""
        return self.gps.fix

    def set_home(self, gps_pose_: List[float]) -> None:
        """Set home origin"""
        if not self.set_origin_cli_.wait_for_service(timeout_sec=3):
            self.__drone.get_logger().error("GPS service not available")
            return

        req = SetOrigin.Request()
        req.origin.latitude = float(gps_pose_[0])
        req.origin.longitude = float(gps_pose_[1])
        req.origin.altitude = float(gps_pose_[2])
        resp = self.set_origin_cli_.call(req)
        if not resp.success:
            self.__drone.get_logger().warn("Origin already set")

    def destroy(self) -> None:
        """Destroy module, clean exit"""
        self.__drone.destroy_subscription(self.gps_sub)

        self.__drone.destroy_client(self.set_origin_cli_)
        # self.__drone.destroy_client(self.global_to_local_cli_)
        # self.__drone.destroy_client(self.local_to_global_cli_)
