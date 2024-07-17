"""A ROS 2 node to publish markers for visualization in RViz."""

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

from collections import deque
import random

from geometry_msgs.msg import PoseStamped, TwistStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from visualization_msgs.msg import Marker


# TODO: choose btter names


class MarkerPublisherNode(Node):
    """A ROS node to publish markers for visualization in RViz."""

    MARKERS_PERIOD = 0.1  # seconds
    MARKERS_LIFETIME = 5  # seconds
    PATH_LIFETIME = 5  # seconds

    def __init__(self) -> None:
        """Initialize the node."""
        super().__init__('marker_publisher_node')

        self.declare_parameter('namespace', 'drone0')
        self.namespace = (
            self.get_parameter('namespace').get_parameter_value().string_value
        )

        self.declare_parameter('record_length', 500)
        record_length = (
            self.get_parameter('record_length').get_parameter_value().integer_value
        )

        self.get_logger().info(f'Using drone namespace: {self.namespace}')

        self.pose_sub = self.create_subscription(
            PoseStamped,
            f'/{self.namespace}/self_localization/pose',
            self.pose_callback,
            qos_profile_sensor_data,
        )
        self.twist_sub = self.create_subscription(
            TwistStamped,
            f'/{self.namespace}/self_localization/twist',
            self.twist_callback,
            qos_profile_sensor_data,
        )
        self.pose_ref_sub = self.create_subscription(
            PoseStamped,
            f'/{self.namespace}/motion_reference/pose',
            self.ref_pose_callback,
            qos_profile_sensor_data,
        )

        self.marker_pub = self.create_publisher(
            Marker, f'/viz/{self.namespace}/reference_pose', qos_profile_system_default
        )
        self.vel_pub = self.create_publisher(
            Marker, f'/viz/{self.namespace}/vel', qos_profile_system_default
        )
        self.path_pub = self.create_publisher(
            Marker, f'/viz/{self.namespace}/last_poses', qos_profile_system_default
        )

        self.poses_record: deque[PoseStamped] = deque(maxlen=record_length)

        self.color = [random.random(), random.random(), random.random()]
        self.marker = Marker()
        self.marker.ns = 'am'
        self.marker.id = 33
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.lifetime = Duration(seconds=self.MARKERS_LIFETIME).to_msg()
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.1
        self.marker.color.r = self.color[0]
        self.marker.color.g = self.color[1]
        self.marker.color.b = self.color[2]
        self.marker.color.a = 1.0

        self.path = Marker()
        self.path.ns = 'am'
        self.path.id = 34
        self.path.type = Marker.POINTS
        self.path.action = Marker.ADD
        self.path.lifetime = Duration(seconds=self.PATH_LIFETIME).to_msg()
        self.path.scale.x = 0.01
        self.path.scale.y = 0.01
        self.path.color.r = self.color[0]
        self.path.color.g = self.color[1]
        self.path.color.b = self.color[2]
        self.path.color.a = 1.0

        self.timer = self.create_timer(self.MARKERS_PERIOD, self.publish_markers)

    def twist_callback(self, msg: TwistStamped) -> None:
        """Twist callback."""
        vel = Marker()

        vel.type = Marker.ARROW
        vel.action = Marker.ADD

        vel.scale.x = 0.1
        vel.scale.y = 0.1
        vel.scale.z = 0.1
        vel.color.r = self.color[0]
        vel.color.g = self.color[1]
        vel.color.b = self.color[2]
        vel.color.a = 1.0
        vel.header = msg.header
        vel.pose.position.x = msg.twist.linear.x
        vel.pose.position.y = msg.twist.linear.y
        vel.pose.position.z = msg.twist.linear.z

        self.vel_pub.publish(vel)

    def pose_callback(self, msg: PoseStamped) -> None:
        """Pose callback."""
        self.poses_record.append(msg)

    def ref_pose_callback(self, msg: PoseStamped) -> None:
        """Motion reference pose callback."""
        self.marker.header = msg.header
        self.marker.pose = msg.pose

        self.marker_pub.publish(self.marker)

    def publish_markers(self) -> None:
        """Publish the markers."""
        if self.poses_record:
            self.path.header = self.poses_record[0].header
            self.path.points = [pose.pose.position for pose in self.poses_record]
            self.path_pub.publish(self.path)


def main(args=None):
    """Entry point."""
    rclpy.init(args=args)
    marker_publisher_node = MarkerPublisherNode()
    rclpy.spin(marker_publisher_node)
    marker_publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
