"""Action handler"""

# Copyright (c) 2022 Universidad Politécnica de Madrid
# All Rights Reserved
#
# Licensed under the BSD-3-Clause (the "License");
# you may not use this file except in compliance with the License.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


__authors__ = "Miguel Fernández Cortizas, Pedro Arias Pérez, David Pérez Saura, Rafael Pérez Seguí"
__copyright__ = "Copyright (c) 2022 Universidad Politécnica de Madrid"
__license__ = "BSD-3-Clause"
__version__ = "0.1.0"


from time import sleep

from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus


class ActionHandler:
    """Action handler"""
    TIMEOUT = 3  # seconds

    class ActionNotAvailable(Exception):
        """Action not available exception"""

    class GoalRejected(Exception):
        """Goal rejected exception"""

    class GoalFailed(Exception):
        """Goal failed exception"""

    def __init__(self, action_client: ActionClient, goal_msg, logger) -> None:
        self._logger = logger

        # Wait for Action availability
        if not action_client.wait_for_server(timeout_sec=self.TIMEOUT):
            raise self.ActionNotAvailable('Action Not Available')

        # Sending goal
        send_goal_future = action_client.send_goal_async(goal_msg,
                                            feedback_callback=self.feedback_callback)

        # Waiting to sending goal result
        while not send_goal_future.done():
            sleep(0.1)

        # Check if goal is accepted
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            raise self.GoalRejected('Goal Rejected')
        self._logger.info('Goal accepted :)')

        # Getting result
        get_result_future = goal_handle.get_result_async()
        while not get_result_future.done():
            sleep(0.1)

        # Check action result
        status = get_result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._logger.info(f"Result: {get_result_future.result().result}")
        else:
            raise self.GoalFailed(f"Goal failed with status code: {status}")

        action_client.destroy()

    def feedback_callback(self, feedback_msg) -> None:
        """feedback callback"""
        self._logger.debug(f'Received feedback: {feedback_msg.feedback}')
