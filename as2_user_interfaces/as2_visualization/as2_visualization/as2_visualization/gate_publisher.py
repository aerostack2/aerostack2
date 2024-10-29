"""A ROS 2 node to publish gate markers for visualization in RViz."""

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
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
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

__authors__ = 'Pedro Arias Pérez'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import argparse

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from visualization_msgs.msg import Marker


class GatePublisherNode(Node):
    """A ROS node to publish gate markers for visualization in RViz."""

    def __init__(self, gate_namespaces: list[str], freq: float, use_sim_time: bool):
        """Initialize the node."""
        super().__init__('gate_marker_publisher_node')

        self.gate_namespaces = gate_namespaces
        if not self.gate_namespaces:
            self.get_logger().error('No gate namespaces provided')
            return

        self.param_use_sim_time = Parameter(
            'use_sim_time', Parameter.Type.BOOL, use_sim_time)
        self.set_parameters([self.param_use_sim_time])

        self.marker_pub = self.create_publisher(
            Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(freq, self.publish_markers)

    def publish_markers(self):
        """Publish the markers."""
        for i, gate_name in enumerate(self.gate_namespaces):
            gate = self.create_gate_marker(gate_name, i)

            self.marker_pub.publish(gate)

    def create_gate_marker(self, gate_name: str, marker_id: int) -> Marker:
        """Create a gate marker at their TF frame."""
        gate = Marker()
        gate.header.stamp = self.get_clock().now().to_msg()
        gate.header.frame_id = gate_name
        gate.ns = 'gate'
        gate.id = marker_id
        gate.type = Marker.MESH_RESOURCE
        gate.mesh_resource = 'package://as2_gazebo_assets/models/gate/meshes/gate.dae'
        gate.mesh_use_embedded_materials = True
        gate.scale.x = 1.0
        gate.scale.y = 1.0
        gate.scale.z = 1.0
        gate.color.a = 1.0
        gate.color.r = 0.5
        gate.color.g = 0.5
        gate.color.b = 0.5
        gate.pose.position.z = 1.45  # see model pose in sdf

        return gate


def main(args=None):
    """Entrypoint method."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--namespace', action='extend', nargs='+', type=str,
                        help='Namespace of gates (ex: --namespace gate0 gate1)')
    parser.add_argument('--freq', type=float, default=1.0,
                        help='Publisher frequency of the markers')
    parser.add_argument('--use_sim_time', action='store_true', help='Use sim time')
    argument_parser = parser.parse_args()

    rclpy.init(args=args)
    marker_publisher_node = GatePublisherNode(
        argument_parser.namespace, argument_parser.freq, argument_parser.use_sim_time)
    rclpy.spin(marker_publisher_node)
    marker_publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
