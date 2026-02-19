#!/usr/bin/env python3

# Copyright 2025 Universidad Politécnica de Madrid
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

"""EKF definition."""

__authors__ = 'Rodrigo da Silva Gómez'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from nav_msgs.msg import Odometry

from ekf_wrapper import EKFWrapper, pose_to_transform
import numpy as np
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from ekf_definition.transform_utils import *
# import tf_transformations as tf


class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')

        # accelerometer_noise_density = 0.0025624546199207194
        # accelerometer_random_walk = 8.055323021637122e-05
        # gyroscope_noise_density = 0.00011090831806067944
        # gyroscope_random_walk = 2.5135360798417067e-06
        # accelerometer_noise_density = 0.0
        # accelerometer_random_walk = 0.0
        # gyroscope_noise_density = 0.0
        # gyroscope_random_walk = 0.0
        self.set_parameters([
            rclpy.parameter.Parameter(
                'use_sim_time',
                rclpy.Parameter.Type.BOOL,
                True),
            # False),  # Set to True if using simulation time
        ])
        accelerometer_noise_density = 1e-1
        accelerometer_random_walk = 1e-2
        gyroscope_noise_density = 1e-2
        gyroscope_random_walk = 1e-3

        # Example parameters
        self.initial_state = np.array([
            0.0, 0.0, 0.0,  # Position (x, y, z)
            0.0, 0.0, 0.0,  # Velocity (vx, vy, vz)
            0.0, 0.0, 0.0,  # Orientation (roll, pitch, yaw)
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0
        ])
        self.initial_covariance = np.zeros((15, 15))
        self.initial_covariance[9:15, 9:15] = np.identity(
            6) * 1  # bias covariance

        print("Initial state:", self.initial_state)
        print("Initial covariance diagonal:", self.initial_covariance)

        self.ekf_wrapper = EKFWrapper(
            self.initial_state,
            self.initial_covariance,
            accelerometer_noise_density,
            gyroscope_noise_density,
            accelerometer_random_walk,
            gyroscope_random_walk)

        print("EKF wrapper initialized.")

        qos_profile = QoSProfile(
            depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)

        self.imu_subscriber = self.create_subscription(
            Imu,
            '/drone0/sensor_measurements/imu',
            self.imu_callback,
            qos_profile
        )
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/drone0/ground_truth/pose',
            self.pose_callback,
            qos_profile
        )
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_publisher = self.create_publisher(
            Odometry,
            '/drone0/odom',
            10
        )
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/drone0/self_localization/pose',
            qos_profile
        )
        self.twist_publisher = self.create_publisher(
            TwistStamped,
            '/drone0/self_localization/twist',
            qos_profile
        )

        self.last_time = 0.0
        self.current_time = 0.0
        self.imu_counter = 0
        self.pose_counter = 0

        self.angular_velocity = np.array([0.0, 0.0, 0.0])

        self.imu_start = np.array([0.0, 0.0, 0.0])
        self.imu_gravity_calibration_number = 200 * 1  # 5 seconds at 200Hz
        self.update_rate = 20  # Hz

        # self.pose_earth_map = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
        self.pose_earth_map = np.array([0.0, 0.0, 0.25, 1.0, 0.0, 0.0, 0.0])

        # Timer 200Hz
        self.timer = self.create_timer(1.0 / 200.0, self.timer_callback)

        # Publish static tfs
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'earth'
        static_transform.child_frame_id = 'drone0/map'
        static_transform.transform.translation.x = self.pose_earth_map[0]
        static_transform.transform.translation.y = self.pose_earth_map[1]
        static_transform.transform.translation.z = self.pose_earth_map[2]
        static_transform.transform.rotation.w = self.pose_earth_map[3]
        static_transform.transform.rotation.x = self.pose_earth_map[4]
        static_transform.transform.rotation.y = self.pose_earth_map[5]
        static_transform.transform.rotation.z = self.pose_earth_map[6]
        self.tf_static_broadcaster.sendTransform(static_transform)
        print("Static transform published: earth -> ekf_map")

    def imu_callback(self, msg):
        # print("IMU callback")
        # print("IMU counter:", self.imu_counter)
        # Process IMU data

        imu_linear_x = msg.linear_acceleration.x
        imu_linear_y = msg.linear_acceleration.y
        imu_linear_z = msg.linear_acceleration.z
        imu_angular_x = msg.angular_velocity.x
        imu_angular_y = msg.angular_velocity.y
        imu_angular_z = msg.angular_velocity.z

        imu_measurement = np.array([
            imu_linear_x,
            imu_linear_y,
            imu_linear_z,
            imu_angular_x,
            imu_angular_y,
            imu_angular_z,
        ])

        self.last_time = self.current_time
        self.current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = self.current_time - self.last_time
        if self.imu_counter == 0:
            dt = 1/200

        if self.imu_counter > self.imu_gravity_calibration_number:
            self.ekf_wrapper.predict(imu_measurement, dt)
            self.angular_velocity = np.array([
                imu_angular_x,
                imu_angular_y,
                imu_angular_z,
            ])
        elif self.imu_counter == self.imu_gravity_calibration_number:
            self.imu_start /= self.imu_counter
            print("IMU start:", self.imu_start)
            self.ekf_wrapper.gravity = self.imu_start
        else:
            # print("IMU counter:", self.imu_counter)
            self.imu_start += np.array([
                imu_linear_x,
                imu_linear_y,
                imu_linear_z,
            ])
        self.imu_counter += 1

    def pose_callback(self, msg):
        # Wait for gravity calibration to finish
        if self.imu_counter > self.imu_gravity_calibration_number:
            # Pose (ground truth) is 200Hz, IMU is 200Hz
            if self.pose_counter % (200 / self.update_rate) == 0:
                # print(f"Update {self.imu_counter}")
                # Process initial pose data
                pose_x = msg.pose.position.x
                pose_y = msg.pose.position.y
                pose_z = msg.pose.position.z
                orientation_x = msg.pose.orientation.x
                orientation_y = msg.pose.orientation.y
                orientation_z = msg.pose.orientation.z
                orientation_w = msg.pose.orientation.w
                orientation = quaternion_to_euler(
                    np.array([orientation_w, orientation_x, orientation_y, orientation_z]))
                # print("Pose orientation (roll, pitch, yaw):", orientation)

                pose_measurement = np.array([
                    pose_x, pose_y, pose_z,
                    orientation[0], orientation[1], orientation[2],
                ])

                self.ekf_wrapper.update_pose(
                    pose_measurement, np.ones(6) * 1e-9)
                # state = self.ekf_wrapper.get_state().T[0]
                # print("State---------------------------------------")
                # print(state[0:3])
                # print(state[3:6])
                # print(state[6:9])
                # print(state[9:12])
                # print(state[12:15])
                # covariance = self.ekf_wrapper.get_state_covariance()
                # print("Covariance diagonal--------------------------")
                # print(np.diag(covariance)[0:3])
                # print(np.diag(covariance)[3:6])
                # print(np.diag(covariance)[6:9])
                # print(np.diag(covariance)[9:12])
                # print(np.diag(covariance)[12:15])
                # exit()
            self.pose_counter += 1

    def timer_callback(self):
        # print("Timer callback")
        stamp = self.get_clock().now().to_msg()
        if self.imu_counter > self.imu_gravity_calibration_number:
            # Get the current state and covariance from the EKF wrapper
            state = self.ekf_wrapper.get_state().T[0]
            covariance = self.ekf_wrapper.get_state_covariance()
            # print("Covariance diagonal--------------------------")
            # print(np.diag(covariance)[9:12])
            # print(np.diag(covariance)[12:15])
            state_position = state[0:3]
            state_rpy = (state[6], state[7], state[8])

            # State is map-base, get transform
            T_map_base = pose_to_transform(state_position, state_rpy)

            # Get the map-odom transformation
            T_map_odom = self.ekf_wrapper.get_map_to_odom()
            pose_map_odom = transform_to_pose(T_map_odom)

            # Compute the odom-base transformation
            T_odom_base = np.linalg.inv(T_map_odom) @ T_map_base
            pose_odom_base = transform_to_pose(T_odom_base)

            # Compute earth-base transformation
            T_earth_base = np.linalg.inv(pose_to_transform(
                self.pose_earth_map[0:3], np.zeros(3))) @ T_map_base
            pose_earth_base = transform_to_pose(T_earth_base)

            # Publish tfs
            # ekf_map -> ekf_odom
            transform_ekf_map_odom = TransformStamped()
            transform_ekf_map_odom.header.stamp = stamp
            transform_ekf_map_odom.header.frame_id = 'drone0/map'
            transform_ekf_map_odom.child_frame_id = 'drone0/odom'
            transform_ekf_map_odom.transform.translation.x = float(
                pose_map_odom[0])
            transform_ekf_map_odom.transform.translation.y = float(
                pose_map_odom[1])
            transform_ekf_map_odom.transform.translation.z = float(
                pose_map_odom[2])
            transform_ekf_map_odom.transform.rotation.w = float(
                pose_map_odom[3])
            transform_ekf_map_odom.transform.rotation.x = float(
                pose_map_odom[4])
            transform_ekf_map_odom.transform.rotation.y = float(
                pose_map_odom[5])
            transform_ekf_map_odom.transform.rotation.z = float(
                pose_map_odom[6])
            self.tf_broadcaster.sendTransform(transform_ekf_map_odom)
            # ekf_odom -> ekf_base_link
            transform_ekf_odom_base = TransformStamped()
            transform_ekf_odom_base.header.stamp = stamp
            transform_ekf_odom_base.header.frame_id = 'drone0/odom'
            transform_ekf_odom_base.child_frame_id = 'drone0'
            transform_ekf_odom_base.transform.translation.x = float(
                pose_odom_base[0])
            transform_ekf_odom_base.transform.translation.y = float(
                pose_odom_base[1])
            transform_ekf_odom_base.transform.translation.z = float(
                pose_odom_base[2])
            transform_ekf_odom_base.transform.rotation.w = float(
                pose_odom_base[3])
            transform_ekf_odom_base.transform.rotation.x = float(
                pose_odom_base[4])
            transform_ekf_odom_base.transform.rotation.y = float(
                pose_odom_base[5])
            transform_ekf_odom_base.transform.rotation.z = float(
                pose_odom_base[6])
            self.tf_broadcaster.sendTransform(transform_ekf_odom_base)

            # Publish the odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = stamp
            odom_msg.header.frame_id = 'drone0/odom'
            odom_msg.child_frame_id = 'drone0/base_link'
            odom_msg.pose.pose.position.x = float(pose_odom_base[0])
            odom_msg.pose.pose.position.y = float(pose_odom_base[1])
            odom_msg.pose.pose.position.z = float(pose_odom_base[2])
            odom_msg.pose.pose.orientation.w = float(pose_odom_base[3])
            odom_msg.pose.pose.orientation.x = float(pose_odom_base[4])
            odom_msg.pose.pose.orientation.y = float(pose_odom_base[5])
            odom_msg.pose.pose.orientation.z = float(pose_odom_base[6])
            diag_covariance = np.double(np.diag(
                np.append(np.diag(covariance)[0:3], np.diag(covariance)[6:9]))).flatten().tolist()
            odom_msg.pose.covariance = diag_covariance
            odom_msg.twist.twist.linear.x = float(state[3])
            odom_msg.twist.twist.linear.y = float(state[4])
            odom_msg.twist.twist.linear.z = float(state[5])
            odom_msg.twist.twist.angular.x = self.angular_velocity[0]
            odom_msg.twist.twist.angular.y = self.angular_velocity[1]
            odom_msg.twist.twist.angular.z = self.angular_velocity[2]
            odom_msg.twist.covariance = np.double(np.eye(6)).flatten().tolist()
            self.odom_publisher.publish(odom_msg)

            # Publish the pose message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = stamp
            pose_msg.header.frame_id = 'earth'
            pose_msg.pose.position.x = float(pose_earth_base[0])
            pose_msg.pose.position.y = float(pose_earth_base[1])
            pose_msg.pose.position.z = float(pose_earth_base[2])
            pose_msg.pose.orientation.w = float(pose_earth_base[3])
            pose_msg.pose.orientation.x = float(pose_earth_base[4])
            pose_msg.pose.orientation.y = float(pose_earth_base[5])
            pose_msg.pose.orientation.z = float(pose_earth_base[6])
            self.pose_publisher.publish(pose_msg)

            # Publish the twist message
            twist_msg = TwistStamped()
            twist_msg.header.stamp = stamp
            twist_msg.header.frame_id = 'earth'
            twist_msg.twist.linear.x = float(state[3])
            twist_msg.twist.linear.y = float(state[4])
            twist_msg.twist.linear.z = float(state[5])
            twist_msg.twist.angular.x = self.angular_velocity[0]
            twist_msg.twist.angular.y = self.angular_velocity[1]
            twist_msg.twist.angular.z = self.angular_velocity[2]
            self.twist_publisher.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)

    ekf_node = EKFNode()

    try:
        rclpy.spin(ekf_node)
    except KeyboardInterrupt:
        pass
    ekf_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
