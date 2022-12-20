"""
gps_module.py
"""

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


__authors__ = "Pedro Arias Pérez, Miguel Fernández Cortizas, David Pérez Saura, Rafael Pérez Seguí"
__copyright__ = "Copyright (c) 2022 Universidad Politécnica de Madrid"
__license__ = "BSD-3-Clause"
__version__ = "0.1.0"

from typing import List, TYPE_CHECKING

from rclpy.qos import qos_profile_sensor_data
from as2_msgs.srv import SetOrigin
# from as2_msgs.srv import GeopathToPath, PathToGeopath
from sensor_msgs.msg import NavSatFix

from as2_python_api.shared_data.gps_data import GpsData
from as2_python_api.modules.module_base import ModuleBase

if TYPE_CHECKING:
    from ..drone_interface import DroneInterface


class GpsModule(ModuleBase):
    """GPS module"""
    __alias__ = "gps"

    def __init__(self, drone: 'DroneInterface') -> None:
        super().__init__(drone, self.__alias__)
        self.__drone = drone

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
        """navdata (gps) callback
        """
        self.gps.fix = [msg.latitude, msg.longitude, msg.altitude]

    @property
    def pose(self) -> List[float]:
        """Get GPS position (lat, lon, alt) in deg and m.

        :rtype: List[float]
        """
        return self.gps.fix

    def set_home(self, gps_pose_: List[float]) -> None:
        """Set home origin
        """
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
        """Destroy module, clean exit
        """
        self.__drone.destroy_subscription(self.gps_sub)

        self.__drone.destroy_client(self.set_origin_cli_)
        # self.__drone.destroy_client(self.global_to_local_cli_)
        # self.__drone.destroy_client(self.local_to_global_cli_)
