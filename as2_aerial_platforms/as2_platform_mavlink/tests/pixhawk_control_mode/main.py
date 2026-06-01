#!/usr/bin/env python3

# Copyright 2023 Universidad Politécnica de Madrid
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
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
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

"""Test Pixhawk control node."""

__authors__ = 'Miguel Fernández Cortizas, Rafael Pérez Seguí, Pedro Arias Pérez'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from time import sleep

from as2_msgs.msg import PlatformControlMode, Thrust
from as2_msgs.srv import SetPlatformControlMode
from geometry_msgs.msg import PoseStamped, TwistStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import SetBool


class Test(Node):
    """Test class."""

    def __init__(self, drone_id='drone_sim_0'):
        super().__init__(f'{drone_id}_test')

        self.offboard_srv = self.create_client(
            SetBool,
            f'{drone_id}/set_offboard_mode')
        if not self.offboard_srv.wait_for_service(timeout_sec=2):
            self.get_logger().error('offboard srv not available')
            exit(1)
        self.arm_srv = self.create_client(
            SetBool,
            f'{drone_id}/set_arming_state')
        if not self.arm_srv.wait_for_service(timeout_sec=2):
            self.get_logger().error('arm service not available')
            exit(1)

        self.cmd_pose_pub = self.create_publisher(
            PoseStamped,
            f'{drone_id}/actuator_command/pose',
            qos_profile_sensor_data)
        self.cmd_twist_pub = self.create_publisher(
            TwistStamped,
            f'{drone_id}/actuator_command/twist',
            qos_profile_sensor_data)
        self.cmd_thrust_pub = self.create_publisher(
            Thrust,
            f'{drone_id}/actuator_command/thrust',
            qos_profile_sensor_data)

        self.set_mode_srv = self.create_client(
            SetPlatformControlMode,
            f'{drone_id}/set_platform_control_mode')

        if not self.set_mode_srv.wait_for_service(timeout_sec=2):
            self.get_logger().error('arm service not available')
            exit(1)

    def arm(self):
        req = SetBool.Request()
        req.data = True
        future = self.arm_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10)
        if not future.done() or not future.result().success:
            self.get_logger().warn('Not armed')

    def offboard(self):
        req = SetBool.Request()
        req.data = True
        future = self.offboard_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10)
        if not future.done() or not future.result().success:
            self.get_logger().warn('Not armed')

    def __set_mode(self, mode, yaw_mode):
        req = SetPlatformControlMode.Request()
        req.control_mode.control_mode = mode
        req.control_mode.yaw_mode = yaw_mode
        # req.control_mode.reference_frame = 0  # Deprecated

        future = self.set_mode_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10)
        if not future.done() or not future.result().success:
            return False
        return True

    def set_position_mode(self, yaw_mode=PlatformControlMode.YAW_ANGLE):
        resp = self.__set_mode(PlatformControlMode.POSITION_MODE, yaw_mode)
        if not resp:
            self.get_logger().warn('POSITION mode not set.')

    def set_velocity_mode(self, yaw_mode=PlatformControlMode.YAW_SPEED):
        resp = self.__set_mode(PlatformControlMode.SPEED_MODE, yaw_mode)
        if not resp:
            self.get_logger().warn('SPEED mode not set.')

    def set_attitude_mode(self, yaw_mode=PlatformControlMode.YAW_ANGLE):
        resp = self.__set_mode(PlatformControlMode.ATTITUDE_MODE, yaw_mode)
        if not resp:
            self.get_logger().warn('ATTITUDE mode not set.')

    def set_acro_mode(self, yaw_mode=PlatformControlMode.YAW_SPEED):
        resp = self.__set_mode(PlatformControlMode.ACRO_MODE, yaw_mode)
        if not resp:
            self.get_logger().warn('ACRO mode not set.')

    def send_pose_command(self, x=0.0, y=0.0, z=0.0):
        msg = PoseStamped()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        # TODO Orientation

        self.cmd_pose_pub.publish(msg)

    def send_speed_command(self, vx=0.0, vy=0.0, vz=0.0, vyaw=0.0):
        msg = TwistStamped()
        msg.header.frame_id = 'odom'
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)

        msg.twist.angular.z = float(vyaw)
        self.cmd_twist_pub.publish(msg)

    def send_attitude_command(self, x=0.0, y=0.0, z=0.0, w=1.0, thrust=0.0):
        msg = PoseStamped()
        msg.header.frame_id = 'odom'
        msg.pose.orientation.x = float(x)
        msg.pose.orientation.y = float(y)
        msg.pose.orientation.z = float(z)
        msg.pose.orientation.w = float(w)
        self.cmd_pose_pub.publish(msg)

        msg2 = Thrust()
        msg2.thrust = float(thrust)
        self.cmd_thrust_pub.publish(msg2)

    def send_acro_command(self, x=0.0, y=0.0, z=0.0, thrust=0.0):
        msg = TwistStamped()
        msg.header.frame_id = 'odom'
        msg.twist.angular.x = float(x)
        msg.twist.angular.y = float(y)
        msg.twist.angular.z = float(z)
        self.cmd_twist_pub.publish(msg)

        msg2 = Thrust()
        msg2.thrust = float(thrust)
        self.cmd_thrust_pub.publish(msg2)


def test_pose_mode():
    test = Test()

    test.offboard()
    print('Offboard')
    sleep(1)

    test.arm()
    print('Armed')
    sleep(1)

    test.set_position_mode()
    print('Position mode')

    test.send_pose_command(z=3.0)
    sleep(3)


def test_speed_mode():
    test = Test()

    test.offboard()
    print('Offboard')
    sleep(1)

    test.arm()
    print('Armed')
    sleep(1)

    test.set_velocity_mode()
    print('Velocity mode')

    test.send_speed_command(vz=5.0)


def test_attitude_mode():
    test = Test()

    test.offboard()
    print('Offboard')
    sleep(1)

    test.arm()
    print('Armed')
    sleep(1)

    test.set_attitude_mode()
    print('Attitude mode')

    test.send_attitude_command(thrust=15.0)
    sleep(3)


def test_acro_mode():
    test = Test()

    test.offboard()
    print('Offboard')
    sleep(1)

    test.arm()
    print('Armed')
    sleep(1)

    test.set_acro_mode()
    print('Acro mode')

    test.send_acro_command(thrust=15.0)
    sleep(3)


def test_1():
    test = Test()

    test.offboard()
    print('Offboard')
    sleep(1)

    test.arm()
    print('Armed')
    sleep(1)

    test.set_position_mode()
    print('Position mode')
    test.send_pose_command(z=3.0)
    sleep(10)

    test.set_velocity_mode()
    print('Velocity mode')
    test.send_speed_command(vx=3.0)
    sleep(10)


def test_2():
    test = Test()

    test.offboard()
    print('Offboard')
    sleep(1)

    test.arm()
    print('Armed')
    sleep(1)

    test.set_position_mode()
    print('Position mode')
    test.send_pose_command(z=3.0)
    sleep(10)

    test.set_velocity_mode()
    print('Velocity mode')
    test.send_speed_command(vyaw=3.0)
    sleep(10)


if __name__ == '__main__':
    rclpy.init()

    # test_pose_mode()
    # test_speed_mode()
    # test_attitude_mode()
    # test_acro_mode()
    test_2()
    print('Bye')

    rclpy.shutdown()
