"""Python interface base.
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

__authors__ = "Miguel Fernández Cortizas, Pedro Arias Pérez, David Pérez Saura, Rafael Pérez Seguí"
__copyright__ = "Copyright (c) 2022 Universidad Politécnica de Madrid"
__license__ = "BSD-3-Clause"
__version__ = "0.1.0"

import threading
from time import sleep
from typing import List, Dict, Union

import rclpy
import rclpy.signals
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from rclpy.parameter import Parameter

from geometry_msgs.msg import PoseStamped, TwistStamped
from as2_msgs.msg import PlatformInfo, AlertEvent

from as2_python_api.shared_data.platform_info_data import PlatformInfoData
from as2_python_api.shared_data.pose_data import PoseData
from as2_python_api.shared_data.twist_data import TwistData

from as2_python_api.service_clients.arming import Arm, Disarm
from as2_python_api.service_clients.offboard import Offboard

from as2_python_api.tools.utils import euler_from_quaternion, get_class_from_module


class DroneInterfaceBase(Node):
    """Drone interface base node"""

    def __init__(self, drone_id: str = "drone0", verbose: bool = False,
                 use_sim_time: bool = False) -> None:
        """Constructor method

        :param drone_id: drone namespace, defaults to "drone0"
        :type drone_id: str, optional
        :param verbose: output mode, defaults to False
        :type verbose: bool, optional
        :param use_sim_time: use simulation time, defaults to False
        :type use_sim_time: bool, optional
        """
        self.modules = {}

        super().__init__(f'{drone_id}_interface', namespace=drone_id)

        self.param_use_sim_time = Parameter(
            'use_sim_time', Parameter.Type.BOOL, use_sim_time)
        self.set_parameters([self.param_use_sim_time])

        self.__executor = rclpy.executors.SingleThreadedExecutor()
        if verbose:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        self.__info = PlatformInfoData()
        self.__pose = PoseData()
        self.__twist = TwistData()

        self.namespace = drone_id
        self.get_logger().info(f"Starting {self.drone_id}")

        self.info_sub = self.create_subscription(
            PlatformInfo, 'platform/info', self.__info_callback, qos_profile_system_default)

        # State subscriber
        self.pose_sub = self.create_subscription(
            PoseStamped, 'self_localization/pose', self.__pose_callback, qos_profile_sensor_data)

        self.twist_sub = self.create_subscription(
            TwistStamped, 'self_localization/twist', self.__twist_callback, qos_profile_sensor_data)

        self.alert_pub = self.create_publisher(
            AlertEvent, "alert_event", qos_profile_system_default)

        self.keep_running = True
        self.__executor.add_node(self)
        self.spin_thread = threading.Thread(target=self.__auto_spin)
        self.spin_thread.start()

        sleep(0.5)
        self.get_logger().info(f'{self.drone_id} interface initialized')

    def __del__(self) -> None:
        self.shutdown()

    # TODO: catch exception if module dont exist
    def load_module(self, pkg: str) -> None:
        """load module on drone"""
        kls = get_class_from_module(pkg)
        if kls.__alias__ in self.modules:
            return  # already loaded

        for dep in kls.__deps__:
            self.load_module(dep)

        setattr(self, kls.__alias__, kls(self))

    @property
    def drone_id(self) -> str:
        """Get drone id (namespace).

        :rtype: str
        """
        return self.namespace

    @property
    def info(self) -> Dict[str, Union[bool, str]]:
        """Get drone info.

        :rtype: Dict[str, Union[bool, str]]
        """
        info = self.__info.data
        return {"connected": info[0], "armed": info[1], "offboard": info[2],
                "state": info[3], "yaw_mode": info[4],
                "control_mode": info[5], "reference_frame": info[6]}

    @property
    def position(self) -> List[float]:
        """Get drone position (x, y, z) in m.

        :rtype: List[float]
        """
        return self.__pose.position

    @property
    def orientation(self) -> List[float]:
        """Get drone orientation (roll, pitch, yaw) in rad.

        :rtype: List[float]
        """
        return self.__pose.orientation

    @property
    def speed(self) -> List[float]:
        """Get drone speed (vx, vy, vz) in m/s.

        :rtype: List[float]
        """
        return self.__twist.twist

    def __info_callback(self, msg: PlatformInfo) -> None:
        """platform info callback"""
        self.__info.data = [msg.connected, msg.armed,
                            msg.offboard, msg.status.state,
                            msg.current_control_mode.yaw_mode,
                            msg.current_control_mode.control_mode,
                            msg.current_control_mode.reference_frame]

    def __pose_callback(self, pose_msg: PoseStamped) -> None:
        """pose stamped callback"""
        self.__pose.position = [pose_msg.pose.position.x,
                                pose_msg.pose.position.y,
                                pose_msg.pose.position.z]

        self.__pose.orientation = [
            *euler_from_quaternion(
                pose_msg.pose.orientation.x,
                pose_msg.pose.orientation.y,
                pose_msg.pose.orientation.z,
                pose_msg.pose.orientation.w)]

    def __twist_callback(self, twist_msg: TwistStamped) -> None:
        """twist stamped callback"""
        self.__twist.twist = [twist_msg.twist.linear.x,
                              twist_msg.twist.linear.y,
                              twist_msg.twist.linear.z]

    def arm(self) -> None:
        """Arm drone.
        """
        sleep(0.1)
        Arm(self)

    def disarm(self) -> None:
        """Disarm drone.
        """
        Disarm(self)

    def offboard(self) -> None:
        """Enable offboard mode.
        """
        Offboard(self)

    # TODO: replace with executor callbacks
    def __auto_spin(self) -> None:
        """Drone inner spin"""
        while self.keep_running and rclpy.ok():
            self.__executor.spin_once(timeout_sec=0)
            sleep(0.05)

    def shutdown(self) -> None:
        """Shutdown properly"""
        self.keep_running = False
        self.destroy_subscription(self.info_sub)
        self.destroy_subscription(self.pose_sub)
        self.destroy_subscription(self.twist_sub)
        self.destroy_publisher(self.alert_pub)

        for module in self.modules.values():
            module.destroy()

        self.modules = {}
        self.spin_thread.join()

    def __send_emergency(self, alert: int) -> None:
        msg = AlertEvent()
        msg.alert = alert
        while rclpy.ok():
            self.alert_pub.publish(msg)
            sleep(0.01)

    def send_emergency_land(self) -> None:
        """Emergency landing"""
        self.get_logger().info("Starting emergency landing")
        self.__send_emergency(AlertEvent.FORCE_LAND)

    def send_emergency_hover(self) -> None:
        """Set controller to hover mode. You will have to take the control manually"""
        self.__send_emergency(AlertEvent.FORCE_HOVER)

    def send_emergency_land_to_aircraft(self) -> None:
        """Call platform emergency land"""
        self.__send_emergency(AlertEvent.EMERGENCY_LAND)

    def send_emergency_hover_to_aircraft(self) -> None:
        """Call platform hover. BE CAREFUL, you will have to take it control manually!"""
        self.__send_emergency(AlertEvent.EMERGENCY_HOVER)

    def send_emergency_killswitch_to_aircraft(self) -> None:
        """Call platform stop. BE CAREFUL, motors will stop!"""
        self.__send_emergency(AlertEvent.KILL_SWITCH)
