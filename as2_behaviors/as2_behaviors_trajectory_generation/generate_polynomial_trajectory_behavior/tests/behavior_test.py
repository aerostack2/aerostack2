#!/usr/bin/env python3

# Copyright 2024 Universidad Politécnica de Madrid
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

"""Test trajectory generator action client."""

__authors__ = 'CVAR'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from as2_msgs.action import TrajectoryGenerator
from as2_msgs.msg import PoseWithID, YawMode
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter


class TrajectoryGeneratorClient(Node):

    def __init__(self):
        super().__init__('trajectory_generator_action_client')
        self.param_use_sim_time = Parameter(
            'use_sim_time', Parameter.Type.BOOL, False)
        self.set_parameters([self.param_use_sim_time])
        self._action_client = ActionClient(
            self, TrajectoryGenerator, '/drone_sim_0/TrajectoryGeneratorBehavior')

    def send_goal(self):
        goal = TrajectoryGenerator.Goal()
        goal.header.frame_id = 'drone_sim_0/odom'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.max_speed = 1.0
        yaw_mode = YawMode()
        yaw_mode.mode = YawMode.FIXED_YAW
        yaw_mode.angle = 0.0
        goal.yaw = yaw_mode

        # pose0 = PoseWithID()
        # pose0.id = '0'
        # pose0.pose.position.x = 0.0
        # pose0.pose.position.y = 0.0
        # pose0.pose.position.z = 1.0
        # goal.path.append(pose0)

        pose1 = PoseWithID()
        pose1.id = '1'
        pose1.pose.position.x = 0.0
        pose1.pose.position.y = 0.0
        pose1.pose.position.z = 2.0
        goal.path.append(pose1)

        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal)


def main(args=None):
    rclpy.init(args=args)
    action_client = TrajectoryGeneratorClient()
    future = action_client.send_goal()
    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()
