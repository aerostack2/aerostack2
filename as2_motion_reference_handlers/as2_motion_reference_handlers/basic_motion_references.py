#!/usr/bin/env python3

"""Implementation of a motion reference handler base."""

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

__authors__ = ' Rafael Perez Seguí, Miguel Fernandez Cortizas, Pedro Arias Perez '
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'
__version__ = '0.1.0'

from dataclasses import dataclass
import time

from as2_msgs.msg import ControllerInfo, ControlMode,  TrajectoryPoint
from as2_msgs.srv import SetControlMode
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import MutuallyExclusiveCallbackGroup, Node
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

as2_names_topic_motion_reference_qos = qos_profile_sensor_data
as2_names_topic_motion_reference_pose = 'motion_reference/pose'
as2_names_topic_motion_reference_twist = 'motion_reference/twist'
as2_names_topic_motion_reference_trajectory = 'motion_reference/trajectory'
as2_names_topic_controller_qos = qos_profile_system_default
as2_names_topic_controller_info = 'controller/info'
as2_names_srv_controller_set_control_mode = 'controller/set_control_mode'


class Singleton(type):
    """Implementation of a singleton class."""

    _instances = {}

    def __call__(cls, *args, **kwargs):
        """Call the singleton class."""
        if cls not in cls._instances:
            cls._instances[cls] = super(
                Singleton, cls).__call__(*args, **kwargs)
        else:
            cls._instances[cls].__init__(*args, **kwargs)
        return cls._instances[cls]


@dataclass
class MotionReferenceHandlerBaseData:
    """Implementation of a motion reference handler base data."""

    command_pose_pub_: Publisher
    command_twist_pub_: Publisher
    command_traj_pub_: Publisher


class BasicMotionReferencesBase(metaclass=Singleton):
    """Implementation of a motion reference handler base for singletons."""

    _instances_list = {}
    number_of_instances_ = -1

    def __init__(self, _node: Node):
        """Construct of the motion reference handler."""
        if _node in self._instances_list:
            _node.get_logger().debug(
                f'Instance of motion reference with {_node.get_namespace()} node already exists')
            return

        _command_pose_pub = _node.create_publisher(
            PoseStamped, as2_names_topic_motion_reference_pose,
            as2_names_topic_motion_reference_qos)

        _command_twist_pub = _node.create_publisher(
            TwistStamped, as2_names_topic_motion_reference_twist,
            as2_names_topic_motion_reference_qos)

        _command_traj_pub = _node.create_publisher(
            TrajectoryPoint, as2_names_topic_motion_reference_trajectory,
            as2_names_topic_motion_reference_qos)

        data = MotionReferenceHandlerBaseData(
            command_pose_pub_=_command_pose_pub,
            command_twist_pub_=_command_twist_pub,
            command_traj_pub_=_command_traj_pub)

        self._instances_list[_node] = data
        _node.get_logger().debug(
            f'Instance of motion reference with {_node.get_namespace()} node created')

    def publish_command_pose(self, _node: Node, _pose: PoseStamped):
        """Publish a pose command."""
        self._instances_list[_node].command_pose_pub_.publish(_pose)

    def publish_command_twist(self, _node: Node, _twist: TwistStamped):
        """Publish a twist command."""
        self._instances_list[_node].command_twist_pub_.publish(_twist)

    def publish_command_trajectory(self, _node: Node, _trajectory: TrajectoryPoint):
        """Publish a trajectory command."""
        self._instances_list[_node].command_traj_pub_.publish(_trajectory)


class BasicMotionReferenceHandler():
    """Implementation of a motion reference handler base."""

    def __init__(self, node: Node):
        """Contructor of the motion reference handler."""
        self.motion_handler_ = BasicMotionReferencesBase(node)
        self.node = node
        self.command_trajectory_msg_ = TrajectoryPoint()
        self.command_pose_msg_ = PoseStamped()
        self.command_twist_msg_ = TwistStamped()
        self.desired_control_mode_ = ControlMode()
        self.desired_control_mode_.yaw_mode = ControlMode.NONE
        self.desired_control_mode_.control_mode = ControlMode.UNSET
        self.desired_control_mode_.reference_frame = ControlMode.UNDEFINED_FRAME

        self.current_mode_ = ControlMode()
        my_callback_group = MutuallyExclusiveCallbackGroup()
        self.controller_info_sub = node.create_subscription(
            ControllerInfo, as2_names_topic_controller_info,
            self.__controller_info_callback,
            as2_names_topic_controller_qos,
            callback_group=my_callback_group)

    def __controller_info_callback(self, msg: ControllerInfo):
        """Call for the controller info topic."""
        self.current_mode_ = msg.input_control_mode

    def check_mode(self) -> bool:
        """Check if the current mode is the desired mode."""
        if (self.desired_control_mode_.control_mode ==
                self.current_mode_.control_mode) == ControlMode.HOVER:
            return True
        if (self.desired_control_mode_.yaw_mode != self.current_mode_.yaw_mode or
                self.desired_control_mode_.control_mode != self.current_mode_.control_mode):
            if not self.__set_mode(self.desired_control_mode_):
                return False
        return True

    def send_pose_command(self) -> bool:
        """Send a pose command."""
        if not self.check_mode():
            return False
        self.command_pose_msg_.header.stamp = self.node.get_clock().now().to_msg()
        self.motion_handler_.publish_command_pose(
            self.node, self.command_pose_msg_)
        return True

    def send_twist_command(self) -> bool:
        """Send a twist command."""
        if not self.check_mode():
            return False
        self.command_twist_msg_.header.stamp = self.node.get_clock().now().to_msg()
        self.motion_handler_.publish_command_twist(
            self.node, self.command_twist_msg_)
        return True

    def send_trajectory_command(self) -> bool:
        """Send a trajectory command."""
        if not self.check_mode():
            return False
        self.motion_handler_.publish_command_trajectory(
            self.node, self.command_trajectory_msg_)
        return True

    def __set_mode(self, mode: ControlMode) -> bool:
        """Set the control mode."""
        set_control_mode_cli_ = self.node.create_client(
            SetControlMode, as2_names_srv_controller_set_control_mode)

        if not set_control_mode_cli_.wait_for_service(timeout_sec=3):
            self.node.get_logger().error(
                f'Service {self.node.get_namespace()}\
                        /{as2_names_srv_controller_set_control_mode} not available')
            return False

        req = SetControlMode.Request()
        req.control_mode = mode
        resp = set_control_mode_cli_.call(req)
        if resp.success:
            init_time = self.node.get_clock().now()
            while self.current_mode_.control_mode != mode.control_mode:
                if (self.node.get_clock().now() - init_time).nanoseconds > 5e9:
                    self.node.get_logger().error(
                        f'Timeout waiting for mode {mode.control_mode}')
                    return False
                time.sleep(0.1)
            self.node.get_logger().debug('Set control mode success')
            return True

        self.node.get_logger().error('Failed to set control mode')
        return False
