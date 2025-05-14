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

__authors__ = 'Guillermo GP-Lenza'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'


from typing import Callable, Generic, TypeVar

from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.node import Node, Publisher, Subscription
from rclpy.qos import QoSProfile
from visualization_msgs.msg import Marker

T = TypeVar('T')
V = TypeVar('V')


class RvizAdapter(Generic[T, V]):
    """
    Base class for RViz Adapters.

    Can be instantiated on the fly by specifiying an adapter function.
    Can also be inherited from to create preset adapters
    """

    def __init__(
        self,
        name: str,
        adapter: Callable[[T], V],
        in_topic: str,
        out_topic: str,
        in_topic_type,
        out_topic_type,
        qos_subscriber: QoSProfile = QoSProfile(depth=10),
        qos_publisher: QoSProfile = QoSProfile(depth=10),
    ):
        """Class constructor for RvizAdapter."""
        self.name = name
        self.in_topic = in_topic
        self.out_topic = out_topic
        self.adapter_f = adapter
        self.qos_subscriber = qos_subscriber
        self.qos_publisher = qos_publisher
        self.in_topic_type = in_topic_type
        self.out_topic_type = out_topic_type


class PointAdapter(RvizAdapter[Point, Marker]):
    """RVizAdapter for points."""

    def __init__(
        self,
        name: str,
        in_topic: str,
        out_topic: str,
        qos_subscriber: QoSProfile = QoSProfile(depth=10),
        qos_publisher: QoSProfile = QoSProfile(depth=10),
    ):
        super().__init__(
            name,
            self.crashing_point_adapter,
            in_topic,
            out_topic,
            Point,
            Marker,
            qos_subscriber,
            qos_publisher,
        )

    @staticmethod
    def crashing_point_adapter(point: Point) -> Marker:
        marker = Marker()
        marker.header.frame_id = 'earth'
        marker.header.stamp.sec = 0
        marker.header.stamp.nanosec = 0
        marker.ns = 'crashing_points'
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.lifetime = Duration(seconds=0).to_msg()
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.4
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.pose.position = point
        return marker


class VizBridge(Node):
    """
    ROS2 Node grouping differents RViz Adapters.

    Each VizBridge is intended to run alone in 1 process.
    """

    def __init__(self, name: str):
        """
        Construct an instance of this class.

        :param name: Node name.
        :type name: str
        """
        super().__init__(name)
        self.adapters: dict[str, tuple[RvizAdapter, Subscription, Publisher]] = {}

    def register_adapter(self, adapter: RvizAdapter[T, V]):
        """
        Register an adapter in the node.

        :param adapter: RVizAdapter to register in the node.
        :type adapter: RVizAdapder
        """
        self.get_logger().info('Registering adapter named ' + adapter.name)
        sub: Subscription = self.create_subscription(
            adapter.in_topic_type,
            adapter.in_topic,
            lambda msg: self.viz_callback(msg, adapter.name),
            adapter.qos_subscriber,
        )
        pub: Publisher = self.create_publisher(
            adapter.out_topic_type, adapter.out_topic, adapter.qos_publisher
        )
        sub_name: str = 'sub_' + str(len(self.adapters))
        pub_name: str = 'pub_' + str(len(self.adapters))

        setattr(self, sub_name, sub)
        setattr(self, pub_name, pub)

        self.adapters[adapter.name] = (adapter, sub, pub)

    def viz_callback(self, msg, name: str):
        """
        Process message using corresponding adapter.

        :param msg: Incoming message
        :param name: Name for the adapter
        :type name: str
        """
        adapter, _, pub = self.adapters[name]
        self.get_logger().info(
            f'Executing callback for adapter {adapter.name} in topic {adapter.out_topic}'
        )
        pub.publish(adapter.adapter_f(msg))
