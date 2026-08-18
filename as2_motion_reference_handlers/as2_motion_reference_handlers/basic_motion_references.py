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
import weakref

from as2_core import as2_names
from as2_msgs.msg import ControllerInfo, ControlMode, PlatformInfo, Thrust, TrajectorySetpoints
from as2_msgs.srv import SetControlMode
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import MutuallyExclusiveCallbackGroup, Node
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from rclpy.subscription import Subscription

COMMAND_QOS = qos_profile_sensor_data
CONTROL_MODE_INFO_QOS = qos_profile_system_default

# When true, references are published to the aerial platform instead of to the
# motion controller, and the control mode is negotiated with the platform
USE_ACTUATOR_COMMANDS_PARAM = 'use_actuator_commands'


def uses_actuator_commands(node: Node) -> bool:
    """
    Read the parameter that sends the references straight to the platform.

    Several handlers share the same node, so the parameter may already be declared.
    """
    if not node.has_parameter(USE_ACTUATOR_COMMANDS_PARAM):
        node.declare_parameter(USE_ACTUATOR_COMMANDS_PARAM, False)
    return node.get_parameter(USE_ACTUATOR_COMMANDS_PARAM).value


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
    """Resources shared by every handler built on the same node."""

    command_pose_pub_: Publisher
    command_twist_pub_: Publisher
    command_traj_pub_: Publisher
    command_thrust_pub_: Publisher
    info_sub_: Subscription
    current_mode_: ControlMode


class BasicMotionReferencesBase(metaclass=Singleton):
    """
    Owner of the resources every handler of a node shares.

    Several handlers on one node talk to the same controller or platform, so they must
    share one set of publishers and, above all, one view of the active control mode: if
    each kept its own, a mode settled by one handler would be unknown to the others.

    Entries are held in a WeakKeyDictionary, so a node that goes away takes its publishers
    with it instead of leaking them for the life of the process.
    """

    _instances_list = weakref.WeakKeyDictionary()

    def __init__(self, _node: Node):
        """
        Build the shared resources of a node, or do nothing if they already exist.

        :param _node: node the references are published from
        :type _node: Node
        """
        if _node in self._instances_list:
            _node.get_logger().debug(
                f'Instance of motion reference with {_node.get_namespace()} node already exists')
            return

        use_actuator_commands = uses_actuator_commands(_node)
        topics = as2_names.topics.actuator_command if use_actuator_commands \
            else as2_names.topics.motion_reference

        _command_pose_pub = _node.create_publisher(
            PoseStamped, topics.pose, COMMAND_QOS)

        _command_twist_pub = _node.create_publisher(
            TwistStamped, topics.twist, COMMAND_QOS)

        _command_traj_pub = _node.create_publisher(
            TrajectorySetpoints, topics.trajectory, COMMAND_QOS)

        _command_thrust_pub = _node.create_publisher(
            Thrust, topics.thrust, COMMAND_QOS)

        data = MotionReferenceHandlerBaseData(
            command_pose_pub_=_command_pose_pub,
            command_twist_pub_=_command_twist_pub,
            command_traj_pub_=_command_traj_pub,
            command_thrust_pub_=_command_thrust_pub,
            info_sub_=None,
            current_mode_=ControlMode())

        # The subscription writes into data, so that every handler of this node reads the
        # same active mode
        callback_group = MutuallyExclusiveCallbackGroup()
        if use_actuator_commands:
            data.info_sub_ = _node.create_subscription(
                PlatformInfo, as2_names.topics.platform.info,
                lambda msg: setattr(data, 'current_mode_', msg.current_control_mode),
                CONTROL_MODE_INFO_QOS, callback_group=callback_group)
        else:
            data.info_sub_ = _node.create_subscription(
                ControllerInfo, as2_names.topics.controller.info,
                lambda msg: setattr(data, 'current_mode_', msg.input_control_mode),
                CONTROL_MODE_INFO_QOS, callback_group=callback_group)

        self._instances_list[_node] = data
        _node.get_logger().debug(
            f'Instance of motion reference with {_node.get_namespace()} node created')

    def get_current_mode(self, _node: Node) -> ControlMode:
        """
        Get the control mode the controller, or the platform, reports as active.

        :param _node: node the handler belongs to
        :type _node: Node
        :return: active control mode
        :rtype: ControlMode
        """
        return self._instances_list[_node].current_mode_

    def set_current_mode(self, _node: Node, _mode: ControlMode):
        """
        Record a control mode just accepted, so every handler of the node sees it.

        :param _node: node the handler belongs to
        :type _node: Node
        :param _mode: mode the controller or platform accepted
        :type _mode: ControlMode
        """
        self._instances_list[_node].current_mode_ = _mode

    def publish_command_pose(self, _node: Node, _pose: PoseStamped):
        """Publish a pose command."""
        self._instances_list[_node].command_pose_pub_.publish(_pose)

    def publish_command_twist(self, _node: Node, _twist: TwistStamped):
        """Publish a twist command."""
        self._instances_list[_node].command_twist_pub_.publish(_twist)

    def publish_command_trajectory(self, _node: Node, _trajectory: TrajectorySetpoints):
        """Publish a trajectory command."""
        self._instances_list[_node].command_traj_pub_.publish(_trajectory)

    def publish_command_thrust(self, _node: Node, _thrust: Thrust):
        """Publish a thrust command."""
        self._instances_list[_node].command_thrust_pub_.publish(_thrust)


class BasicMotionReferenceHandler():
    """
    Base of the motion reference handlers: publish references and negotiate the mode.

    References go to the motion controller, unless the ROS 2 parameter
    use_actuator_commands of the node is true: then they go straight to the aerial
    platform, which is also the one the control mode is negotiated with. The platform must
    support the control modes the references need, since there is no controller in between
    to synthesize them.
    """

    def __init__(self, node: Node):
        """
        Create the handler, its publishers and the control mode subscription it selects.

        :param node: node the references are published from
        :type node: Node
        """
        self.motion_handler_ = BasicMotionReferencesBase(node)
        self.node = node
        self.command_trajectory_msg_ = TrajectorySetpoints()
        self.command_pose_msg_ = PoseStamped()
        self.command_twist_msg_ = TwistStamped()
        self.desired_control_mode_ = ControlMode()
        self.desired_control_mode_.yaw_mode = ControlMode.NONE
        self.desired_control_mode_.control_mode = ControlMode.UNSET

        self.command_thrust_msg_ = Thrust()

        self.use_actuator_commands_ = uses_actuator_commands(node)

    def check_mode(self) -> bool:
        """
        Settle the desired control mode if the active one does not match it.

        Hovering does not need to be settled again, whatever its yaw mode is.

        :return: True if the desired control mode is the active one
        :rtype: bool
        """
        # Hovering does not need to be settled again, whatever its yaw mode is
        current_mode = self.motion_handler_.get_current_mode(self.node)
        if (self.desired_control_mode_.control_mode ==
                current_mode.control_mode == ControlMode.HOVER):
            return True
        if (self.desired_control_mode_.yaw_mode != current_mode.yaw_mode or
                self.desired_control_mode_.control_mode != current_mode.control_mode):
            if not self.__set_mode(self.desired_control_mode_):
                return False
        return True

    def check_frame_id(self, frame_id: str, reference: str) -> bool:
        """
        Check that a reference carries the frame its data is expressed in.

        Without a frame id the reference cannot be converted, and whoever receives it
        would act on it as if it were already in its own frame.

        :param frame_id: frame id of the reference message
        :type frame_id: str
        :param reference: name of the reference, for the error message
        :type reference: str
        :return: True if the reference can be sent
        :rtype: bool
        """
        if frame_id:
            return True
        # Without a frame id the reference cannot be converted, and whoever
        # receives it would act on it as if it were already in its own frame
        self.node.get_logger().error(
            f'Not sending {reference} reference without frame_id')
        return False

    def send_pose_command(self) -> bool:
        """
        Send the current pose reference, settling the control mode first.

        :return: True if the reference was published, False if it has no frame_id
        :rtype: bool
        """
        if not self.check_frame_id(self.command_pose_msg_.header.frame_id, 'pose'):
            return False
        if not self.check_mode():
            return False
        self.command_pose_msg_.header.stamp = self.node.get_clock().now().to_msg()
        self.motion_handler_.publish_command_pose(
            self.node, self.command_pose_msg_)
        return True

    def send_twist_command(self) -> bool:
        """
        Send the current twist reference, settling the control mode first.

        :return: True if the reference was published, False if it has no frame_id
        :rtype: bool
        """
        if not self.check_frame_id(self.command_twist_msg_.header.frame_id, 'twist'):
            return False
        if not self.check_mode():
            return False
        self.command_twist_msg_.header.stamp = self.node.get_clock().now().to_msg()
        self.motion_handler_.publish_command_twist(
            self.node, self.command_twist_msg_)
        return True

    def send_trajectory_command(self) -> bool:
        """
        Send the current trajectory reference, settling the control mode first.

        :return: True if the reference was published, False if it has no frame_id
        :rtype: bool
        """
        if not self.check_frame_id(self.command_trajectory_msg_.header.frame_id, 'trajectory'):
            return False
        if not self.check_mode():
            return False
        self.motion_handler_.publish_command_trajectory(
            self.node, self.command_trajectory_msg_)
        return True

    def send_thrust_command(self) -> bool:
        """
        Send the current thrust reference, settling the control mode first.

        :return: True if the reference was published
        :rtype: bool
        """
        if not self.check_mode():
            return False
        self.command_thrust_msg_.header.stamp = self.node.get_clock().now().to_msg()
        self.motion_handler_.publish_command_thrust(
            self.node, self.command_thrust_msg_)
        return True

    def __set_mode(self, mode: ControlMode) -> bool:
        """
        Set the control mode.

        The platform validates the mode against the ones it supports, the
        controller also looks for a way of synthesizing it with its plugin.
        """
        service_name = as2_names.services.platform.set_platform_control_mode \
            if self.use_actuator_commands_ else as2_names.services.controller.set_control_mode
        set_control_mode_cli_ = self.node.create_client(SetControlMode, service_name)

        if not set_control_mode_cli_.wait_for_service(timeout_sec=3):
            self.node.get_logger().error(
                f'Service {self.node.get_namespace()}/{service_name} not available')
            return False

        req = SetControlMode.Request()
        req.control_mode = mode
        resp = set_control_mode_cli_.call(req)
        if resp.success:
            init_time = self.node.get_clock().now()
            while self.motion_handler_.get_current_mode(
                    self.node).control_mode != mode.control_mode:
                if (self.node.get_clock().now() - init_time).nanoseconds > 5e9:
                    self.node.get_logger().error(
                        f'Timeout waiting for mode {mode.control_mode}')
                    return False
                time.sleep(0.1)
            self.node.get_logger().debug('Set control mode success')
            return True

        self.node.get_logger().error('Failed to set control mode')
        return False
