from typing import Callable, TypeVar, Generic
from rclpy.node import Node, Subscription, Publisher
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from emlanding_msgs.msg import AvailableAreas
from emlanding_msgs.msg import Route

T = TypeVar("T")
V = TypeVar("V")


class VizBridge(Node):

    def __init__(self, name: str, drone_id: str):
        super().__init__(name)
        self.adapters: dict[str, tuple[RvizAdapter, Subscription, Publisher]] = {}
        self.drone_id: str = drone_id

    def register_adapter(self, adapter: "RvizAdapter[T, V]"):
        self.sub: Subscription = self.create_subscription(
            T,
            adapter.in_topic,
            lambda msg: self.viz_callback(msg, adapter.name),
            adapter.qos_subscriber,
        )
        self.pub: Publisher = self.create_publisher(
            V, adapter.out_topic, adapter.qos_publisher
        )
        self.adapters[adapter.name] = (adapter, self.sub, self.pub)

    def viz_callback(self, msg, name: str):
        adapter, _, pub = self.adapters[name]
        pub.publish(adapter.adapter_f(msg))


class RvizAdapter(Generic[T, V]):

    def __init__(
        self,
        name: str,
        adapter: Callable[[T], V],
        in_topic: str,
        out_topic: str,
        qos_subscriber: QoSProfile = QoSProfile(depth=10),
        qos_publisher: QoSProfile = QoSProfile(depth=10),
    ):
        self.name = name
        self.in_topic = in_topic
        self.out_topic = out_topic
        self.adapter_f = adapter
        self.qos_subscriber = qos_subscriber
        self.qos_publisher = qos_publisher


class CrashingPointAdapter(RvizAdapter[Point, Marker]):

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
            qos_subscriber,
            qos_publisher,
        )

    def crashing_point_adapter(self, point: Point) -> Marker:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp.sec = 0
        marker.header.stamp.nanosec = 0
        marker.ns = "crashing_points"
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


class LandingAreasAdapter(RvizAdapter[AvailableAreas, MarkerArray]):
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
            self.landing_areas_adapter,
            in_topic,
            out_topic,
            qos_subscriber,
            qos_publisher,
        )

    def landing_areas_adapter(self, areas: AvailableAreas) -> MarkerArray:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp.sec = 0
        marker.header.stamp.nanosec = 0
        marker.ns = "landing_areas"
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.lifetime = Duration(seconds=0).to_msg()
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.01
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.pose.position.z = 0.0
        ma_msg = MarkerArray()
        ma: list[Marker] = []
        for la in areas.possiblelandings:
            marker.pose.position = la.destination
            ma.append(marker)

        ma_msg.markers = ma
        return ma_msg


class TrajectoryAdapter(RvizAdapter[Route, Marker]):
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
            self.trajectory_adapter,
            in_topic,
            out_topic,
            qos_subscriber,
            qos_publisher,
        )

    def trajectory_adapter(self, msg: Route) -> Marker:

        marker = Marker()
        marker.header.stamp.sec = 0
        marker.header.stamp.nanosec = 0
        marker.header.frame_id = "earth"
        marker.ns = "planned_trajectories"
        marker.type = Marker.LINE_STRIP
        marker.id = 14
        marker.action = Marker.ADD
        marker.lifetime = Duration(seconds=0).to_msg()
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        m = max(msg.route, key=lambda x: x.speed)

        route = msg.route

        for i in range(msg.route):  # type:ignore
            v = msg.route[i].speed  # type:ignore
            color_p: ColorRGBA = ColorRGBA()
            color_p.r = float(1 - (float(v) / m))
            color_p.g = float(v) / m
            color_p.b = float(0)
            color_p.a = 1.0
            marker.colors.append(color_p)  # type:ignore

            viz_p: Point = msg.route[i]  # type:ignore
            marker.points.append(viz_p)  # type:ignore

        return marker
