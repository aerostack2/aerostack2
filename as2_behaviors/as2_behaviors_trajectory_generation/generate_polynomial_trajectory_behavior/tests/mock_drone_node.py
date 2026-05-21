#!/usr/bin/env python3

# Copyright 2026 Universidad Politécnica de Madrid
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
r"""
Mock support node for the polynomial trajectory generator behavior.

Replaces the rest of the aerostack2 stack so the behavior can be exercised
end-to-end with only the action client. Intended to be launched manually
together with the behavior:

    ros2 launch as2_behaviors_trajectory_generation \
        generate_polynomial_trajectory_behavior_launch.py namespace:=drone0

    python3 mock_drone_node.py        # this file
    python3 behavior_test.py          # the action client

Provided by this node (all under ``/{NAMESPACE}/``):

* TF tree ``earth → {ns}/map → {ns}/odom → {ns}/base_link``. The first two
  links are static identity. The ``odom → base_link`` link is dynamic: its
  pose tracks the latest ``motion_reference/trajectory`` setpoint received.
* Service ``controller/set_control_mode`` (``as2_msgs/srv/SetControlMode``)
  that always succeeds and echoes the requested mode on ``controller/info``.
* Periodic ``controller/info`` (``as2_msgs/ControllerInfo``) so late
  subscribers see the latest mode.
* Periodic ``self_localization/twist`` (``geometry_msgs/TwistStamped``).
  Twist values mirror the latest setpoint twist (zeros until the first
  setpoint arrives).

Frame and unit conventions:

* Setpoint position/twist are taken in the behavior's ``desired_frame_id``
  (``{ns}/odom`` by default). The mock applies them as the
  ``odom → base_link`` transform and as the published twist (header
  ``frame_id`` set to ``{ns}/odom`` to match).
* Yaw is encoded as a Z-axis quaternion in the TF.

Tweak the configuration block at the top of this file to change namespace,
initial pose, or rates.
"""

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2026 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import math
import sys
import threading

from as2_msgs.msg import ControllerInfo, ControlMode
from as2_msgs.msg import TrajectorySetpoints
from as2_msgs.srv import SetControlMode

from geometry_msgs.msg import TransformStamped, TwistStamped

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, ReliabilityPolicy

from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


# ───────────────────────────── Configuration ────────────────────────────────
NAMESPACE = 'drone0'

# Initial state of the simulated vehicle (before any setpoint is received).
INITIAL_X = 0.0     # m, in {NAMESPACE}/odom
INITIAL_Y = 0.0     # m
INITIAL_Z = 0.0     # m
INITIAL_YAW = 0.0   # rad, around Z

# Publishing rates
TF_DYNAMIC_RATE_HZ = 100.0
TWIST_RATE_HZ = 100.0
CONTROLLER_INFO_RATE_HZ = 10.0
# ───────────────────────────────────────────────────────────────────────────


def yaw_to_quaternion_z(yaw: float):
    """Return (qx, qy, qz, qw) for a rotation of ``yaw`` rad around +Z."""
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


class MockDroneNode(Node):
    """
    Mock of the rest of the aerostack2 stack required by the behavior.

    Holds the latest commanded vehicle state behind a lock and republishes
    it on TF and ``self_localization/twist`` on fixed-rate timers. The lock
    is required because the trajectory subscription, the service callback
    and the timer callbacks may all run concurrently under a multi-threaded
    executor — even though the default executor is single-threaded the lock
    keeps the contract explicit and cheap.
    """

    def __init__(self):
        """Set up TF, pub/sub, the service and the periodic timers."""
        super().__init__('mock_drone_support', namespace=NAMESPACE)

        self._lock = threading.Lock()
        self._pose_x = float(INITIAL_X)
        self._pose_y = float(INITIAL_Y)
        self._pose_z = float(INITIAL_Z)
        self._pose_yaw = float(INITIAL_YAW)
        self._twist_lin = [0.0, 0.0, 0.0]
        self._twist_ang = [0.0, 0.0, 0.0]
        self._control_mode = ControlMode()  # all-zero = UNSET / NONE

        self._ns = self.get_namespace().lstrip('/')
        self._earth_frame = 'earth'
        self._map_frame = f'{self._ns}/map'
        self._odom_frame = f'{self._ns}/odom'
        self._base_link_frame = f'{self._ns}/base_link'

        # TF: two static identity links and one dynamic odom → base_link.
        self._static_tf = StaticTransformBroadcaster(self)
        self._publish_static_transforms()
        self._tf_broadcaster = TransformBroadcaster(self)

        # Self-localization twist (matches as2_names::topics::self_localization::qos
        # which is SensorDataQoS).
        self._twist_pub = self.create_publisher(
            TwistStamped, 'self_localization/twist', qos_profile_sensor_data)

        # Controller info: the motion reference handlers subscribe to this to
        # confirm the mode they requested via set_control_mode is active.
        info_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._info_pub = self.create_publisher(
            ControllerInfo, 'controller/info', info_qos)

        # Trajectory setpoints arriving from the behavior's trajectory motion
        # handler. We mirror them onto the TF and twist topics. The publisher
        # uses SensorDataQoS (BEST_EFFORT, depth 10), so we must match it on
        # the subscription side or the messages are dropped on QoS mismatch.
        self._traj_sub = self.create_subscription(
            TrajectorySetpoints, 'motion_reference/trajectory',
            self._on_trajectory, qos_profile_sensor_data)

        # set_control_mode service: always succeeds and immediately echoes
        # the requested mode on controller/info.
        self._mode_srv = self.create_service(
            SetControlMode, 'controller/set_control_mode',
            self._on_set_control_mode)

        # Periodic publishers
        self.create_timer(1.0 / TF_DYNAMIC_RATE_HZ, self._publish_dynamic_tf)
        self.create_timer(1.0 / TWIST_RATE_HZ, self._publish_twist)
        self.create_timer(
            1.0 / CONTROLLER_INFO_RATE_HZ, self._publish_controller_info)

        self.get_logger().info(
            f'Mock drone support up: ns="{self._ns}" '
            f'tf_root="{self._earth_frame}" base="{self._base_link_frame}"')

    def _publish_static_transforms(self) -> None:
        """Send identity ``earth→map`` and ``map→odom`` transforms once."""
        stamp = self.get_clock().now().to_msg()
        transforms = []
        for parent, child in (
            (self._earth_frame, self._map_frame),
            (self._map_frame, self._odom_frame),
        ):
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = parent
            tf.child_frame_id = child
            tf.transform.rotation.w = 1.0
            transforms.append(tf)
        self._static_tf.sendTransform(transforms)

    def _publish_dynamic_tf(self) -> None:
        """Broadcast the current ``odom → base_link`` transform."""
        with self._lock:
            x, y, z, yaw = (
                self._pose_x, self._pose_y, self._pose_z, self._pose_yaw)
        qx, qy, qz, qw = yaw_to_quaternion_z(yaw)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self._odom_frame
        tf.child_frame_id = self._base_link_frame
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(tf)

    def _publish_twist(self) -> None:
        """
        Publish the current twist on ``self_localization/twist``.

        The twist values are taken verbatim from the last setpoint, in the
        behavior's desired frame (``{ns}/odom``). The header ``frame_id``
        matches that frame so the behavior's TF-aware state callback can
        consume it directly.
        """
        with self._lock:
            lin = list(self._twist_lin)
            ang = list(self._twist_ang)
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._odom_frame
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = lin
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = ang
        self._twist_pub.publish(msg)

    def _publish_controller_info(self) -> None:
        """Publish the most recently set control mode on ``controller/info``."""
        with self._lock:
            mode = self._control_mode
        msg = ControllerInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.input_control_mode = mode
        msg.output_control_mode = mode
        self._info_pub.publish(msg)

    def _on_trajectory(self, msg: TrajectorySetpoints) -> None:
        """
        Copy the first setpoint into the simulated vehicle state.

        The behavior's trajectory motion handler always populates at least
        one setpoint per message; we ignore the rest. We assume the message
        is expressed in ``{ns}/odom`` (the behavior's ``desired_frame_id``).
        """
        if not msg.setpoints:
            return
        sp = msg.setpoints[0]
        with self._lock:
            self._pose_x = float(sp.position.x)
            self._pose_y = float(sp.position.y)
            self._pose_z = float(sp.position.z)
            self._pose_yaw = float(sp.yaw_angle)
            self._twist_lin = [
                float(sp.twist.x), float(sp.twist.y), float(sp.twist.z)]
            # Trajectory setpoints carry no angular velocity; keep it zero.
            self._twist_ang = [0.0, 0.0, 0.0]

    def _on_set_control_mode(self, request, response):
        """Accept the requested mode, store it and echo it on ``controller/info``."""
        with self._lock:
            self._control_mode = request.control_mode
        response.success = True
        # Immediate echo so the calling motion handler sees the new mode
        # without waiting for the next periodic publish.
        info = ControllerInfo()
        info.header.stamp = self.get_clock().now().to_msg()
        info.input_control_mode = request.control_mode
        info.output_control_mode = request.control_mode
        self._info_pub.publish(info)
        self.get_logger().info(
            f'set_control_mode: mode={request.control_mode.control_mode} '
            f'yaw={request.control_mode.yaw_mode} '
            f'frame={request.control_mode.reference_frame}')
        return response


def main():
    """Spin the mock until interrupted."""
    rclpy.init(args=sys.argv)
    node = MockDroneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
