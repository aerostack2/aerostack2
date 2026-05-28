"""Swarm configurator — formation builders and SwarmFlocking action client."""

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

__authors__ = 'jongmin lee'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import math
from typing import List

from as2_msgs.action import SwarmFlocking
from as2_msgs.msg import PoseWithID
from geometry_msgs.msg import Pose, PoseStamped

from rclpy.action import ActionClient
from rclpy.node import Node


def build_line_formation(namespaces: List[str], spacing: float = 1.0) -> List[PoseWithID]:
    """Build a centered line formation along the X axis.

    :param namespaces: Ordered list of drone namespaces.
    :param spacing: Distance between adjacent drones in metres, defaults to 1.0
    :return: List of PoseWithID relative to the virtual centroid.
    """
    n = len(namespaces)
    formation = []
    for i, ns in enumerate(namespaces):
        pwi = PoseWithID()
        pwi.id = ns
        pwi.pose = Pose()
        pwi.pose.position.x = (i - (n - 1) / 2.0) * spacing
        pwi.pose.position.y = 0.0
        pwi.pose.position.z = 0.0
        pwi.pose.orientation.w = 1.0
        formation.append(pwi)
    return formation


def build_circle_formation(namespaces: List[str], radius: float = 2.0) -> List[PoseWithID]:
    """Build a circle formation centred at the virtual centroid.

    :param namespaces: Ordered list of drone namespaces.
    :param radius: Circle radius in metres, defaults to 2.0
    :return: List of PoseWithID relative to the virtual centroid.
    """
    n = len(namespaces)
    formation = []
    for i, ns in enumerate(namespaces):
        angle = 2.0 * math.pi * i / n
        pwi = PoseWithID()
        pwi.id = ns
        pwi.pose = Pose()
        pwi.pose.position.x = radius * math.cos(angle)
        pwi.pose.position.y = radius * math.sin(angle)
        pwi.pose.position.z = 0.0
        pwi.pose.orientation.w = 1.0
        formation.append(pwi)
    return formation


class SwarmConfigurator:
    """Sends SwarmFlocking action goals to the swarm behavior server.

    Attaches an ActionClient to the provided Node so no separate node is needed.
    """

    FORMATION_BUILDERS = {
        'line': build_line_formation,
        'circle': build_circle_formation,
    }

    def __init__(self, node: Node, server_name: str) -> None:
        """
        :param node: ROS2 node that owns the action client.
        :param server_name: Full action server name, e.g. '/Swarm/SwarmFlockingBehavior'.
        """
        self._node = node
        self._client = ActionClient(node, SwarmFlocking, server_name)

    def send(
        self,
        drone_namespaces: List[str],
        virtual_centroid: PoseStamped,
        formation: List[PoseWithID],
    ) -> bool:
        """Send a SwarmFlocking goal.

        :param drone_namespaces: Namespaces of drones to include in the swarm.
        :param virtual_centroid: Reference pose for the swarm centre.
        :param formation: Per-drone offsets relative to the virtual centroid.
        :return: True if the goal was sent successfully.
        """
        if not self._client.wait_for_server(timeout_sec=3.0):
            self._node.get_logger().error(
                f'[SwarmConfigurator] Action server not available: {self._client._action_name}')
            return False

        goal = SwarmFlocking.Goal()
        goal.virtual_centroid = virtual_centroid
        goal.swarm_formation = formation
        goal.drones_namespace = drone_namespaces

        self._client.send_goal_async(goal)
        self._node.get_logger().info(
            f'[SwarmConfigurator] Goal sent — drones: {drone_namespaces}')
        return True

    def send_with_default_formation(
        self,
        drone_namespaces: List[str],
        virtual_centroid: PoseStamped,
        formation_type: str = 'line',
        spacing: float = 1.0,
        radius: float = 2.0,
    ) -> bool:
        """Build a formation automatically and send the goal.

        :param drone_namespaces: Namespaces of drones to include.
        :param virtual_centroid: Reference pose for the swarm centre.
        :param formation_type: 'line' or 'circle', defaults to 'line'.
        :param spacing: Spacing for line formation in metres, defaults to 1.0.
        :param radius: Radius for circle formation in metres, defaults to 2.0.
        :return: True if the goal was sent successfully.
        """
        builder = self.FORMATION_BUILDERS.get(formation_type)
        if builder is None:
            self._node.get_logger().error(
                f'[SwarmConfigurator] Unknown formation type: {formation_type}. '
                f'Available: {list(self.FORMATION_BUILDERS.keys())}')
            return False

        if formation_type == 'line':
            formation = builder(drone_namespaces, spacing)
        else:
            formation = builder(drone_namespaces, radius)

        return self.send(drone_namespaces, virtual_centroid, formation)
