"""
behavior_handler.py
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

import abc
from time import sleep

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile

from as2_msgs.msg import BehaviorStatus
from std_srvs.srv import Trigger

from action_msgs.msg import GoalStatus


class BehaviorHandler(abc.ABC):
    """Behavior handler"""
    TIMEOUT = 1  # seconds

    __goal_handle = None
    __result = None

    class BehaviorNotAvailable(Exception):
        """Behavior not available exception"""

    class GoalRejected(Exception):
        """Goal rejected exception"""

    class ResultUnknown(Exception):
        """Result unknown exception"""

    def __init__(self, node: 'Node', action_msg, behavior_name) -> None:
        self._node = node
        self.__status = BehaviorStatus.IDLE
        self.__feedback = None

        self.__action_client = ActionClient(node, action_msg, behavior_name)

        self.__pause_client = self._node.create_client(
            Trigger, behavior_name + "/_behavior/pause")
        self.__resume_client = self._node.create_client(
            Trigger, behavior_name + "/_behavior/resume")
        self.__stop_client = self._node.create_client(
            Trigger, behavior_name + "/_behavior/stop")

        self.__status_sub = self._node.create_subscription(
            BehaviorStatus, behavior_name + "/_behavior/behavior_status",
            self.__status_callback, QoSProfile(depth=1))

        # Wait for Action and Servers availability
        if not self.__action_client.wait_for_server(timeout_sec=self.TIMEOUT) or \
                not self.__pause_client.wait_for_service(timeout_sec=self.TIMEOUT) or \
                not self.__resume_client.wait_for_service(timeout_sec=self.TIMEOUT) or \
                not self.__stop_client.wait_for_service(timeout_sec=self.TIMEOUT):
            raise self.BehaviorNotAvailable(f'{behavior_name} Not Available')

    def destroy(self) -> None:
        """Clean exit
        """
        self._node.destroy_subscription(self.__status_sub)
        self._node.destroy_client(self.__resume_client)
        self._node.destroy_client(self.__pause_client)
        self.__action_client.destroy()

    @property
    def status(self) -> int:
        """Behavior status

        :return: IDLE, PAUSED, RUNNING
        :rtype: int
        """
        return self.__status

    @property
    def feedback(self):
        """Behavior feedback

        :return: rclpy.Feedback
        """
        return self.__feedback

    @property
    def result_status(self):
        """Behavior result status

        :return: rclpy.GoalStatus
        """
        try:
            return BehaviorHandler.__result.status
        except AttributeError:
            return GoalStatus.STATUS_UNKNOWN

    @property
    def result(self):
        """Behavior result

        :raises self.ResultUnknown: on result not ready
        :return: rclpy.Result
        """
        if self.result_status not in [GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED]:
            raise self.ResultUnknown("Result not received yet")
        return BehaviorHandler.__result.result

    def start(self, goal_msg, wait_result: bool = True) -> bool:
        """Start behavior

        :param goal_msg: behavior goal
        :type goal_msg: Goal
        :param wait_result: wait to behavior end, defaults to True
        :type wait_result: bool, optional
        :raises self.GoalRejected: on goal rejection
        :return: succeeded or not
        :rtype: bool
        """
        # Sending goal
        send_goal_future = self.__action_client.send_goal_async(goal_msg,
                                                                feedback_callback=self.__feedback_callback)

        # Waiting to sending goal result
        while not send_goal_future.done():
            sleep(0.1)

        # Check if goal is accepted
        BehaviorHandler.__goal_handle = send_goal_future.result()
        if not BehaviorHandler.__goal_handle.accepted:
            raise self.GoalRejected('Goal Rejected')

        if wait_result:
            return self.wait_to_result()

        return True

    # TODO
    def modify(self, goal_msg):
        raise NotImplementedError

    def pause(self) -> bool:
        """Pause current behavior

        :return: pause succeed or not
        :rtype: bool
        """
        # TODO: extend to all behavior status
        if self.status != BehaviorStatus.RUNNING:
            return True
        response = self.__pause_client.call(Trigger.Request())
        return response.success

    def resume(self, wait_result: bool = True) -> bool:
        """Continue with current behavior

        :param wait_result: wait to behavior end, defaults to True
        :type wait_result: bool, optional
        :return: resume succeed or not
        :rtype: bool
        """
        # TODO: extend to all behavior status
        if self.status != BehaviorStatus.PAUSED:
            return True
        response = self.__resume_client.call(Trigger.Request())
        if wait_result:
            return self.wait_to_result()
        return response.success

    def stop(self) -> bool:
        """Stop current behavior

        :return: stop succeed or not
        :rtype: bool
        """
        if self.status != BehaviorStatus.RUNNING:
            return True
        response = self.__stop_client.call(Trigger.Request())
        return response.success

    def wait_to_result(self) -> bool:
        """Wait to inner action to finish

        :raises GoalFailed: When behaviour result not succeeded
        :return: succeeded or not
        :rtype: bool
        """
        if self.status == BehaviorStatus.IDLE:
            return True

        # Getting result
        result_future = BehaviorHandler.__goal_handle.get_result_async()
        while not result_future.done():
            sleep(0.1)

        # Check action result
        BehaviorHandler.__result = result_future.result()

        if self.result_status != GoalStatus.STATUS_SUCCEEDED:
            self._node.get_logger().debug(
                f"Goal failed with status code: {self.result_status}")
            return False
        self._node.get_logger().debug(f"Result: {self.result}")
        return True

    def __feedback_callback(self, feedback_msg) -> None:
        """feedback callback
        """
        self.__feedback = feedback_msg.feedback
        self._node.get_logger().debug(
            f'Received feedback: {feedback_msg.feedback}')

    def __status_callback(self, status_msg: BehaviorStatus) -> None:
        """behavior status callback
        """
        self.__status = status_msg.status
