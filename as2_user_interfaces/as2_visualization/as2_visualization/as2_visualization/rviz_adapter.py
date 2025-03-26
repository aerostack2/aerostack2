from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from typing import Callable, TypeVar, Generic
from rclpy.duration import Duration
from geometry_msgs.msg import Point
from emlanding_msgs.msg import AvailableAreas   
from emlanding_msgs.msg import Route 
from std_msgs.msg import ColorRGBA

T = TypeVar('T')
V = TypeVar('V')

class RvizAdapter(Node, Generic[T,V]):

    def __init__(self, name:str, adapter : Callable[[T], V], in_topic:str, in_msg : T, out_topic:str, out_msg : V):
        super().__init__(name)
        self.in_topic = in_topic
        self.out_topic = out_topic
        self.adapter_f = adapter
        self.sub = self.create_subscription(in_msg, self.in_topic, self.marker_callback, 10)
        self.pub = self.create_publisher(out_msg, self.out_topic, 10)

    def marker_callback(self, msg): 
        m = self.adapter_f(msg)
        self.pub.publish(m)

class CrashingPointAdapter(RvizAdapter):

    def __init__(self, name : str, in_topic : str, out_topic : str):
        super().__init__(name, self.crashing_point_adapter, in_topic, Point, out_topic, Marker)

    def crashing_point_adapter(self, point : Point) -> Marker:
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
    
class LandingAreasAdapter(RvizAdapter):
    def __init__(self, name : str, in_topic : str, out_topic : str):
        super().__init__(name, self.landing_areas_adapter, in_topic, AvailableAreas, out_topic, MarkerArray)

    def landing_areas_adapter(self, areas : AvailableAreas) -> MarkerArray:
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
        ma : list[Marker] = []
        for la in areas.possiblelandings:
            marker.pose.position = la.destination
            ma.append(marker)
        
        ma_msg.markers = ma
        return ma_msg

class TrajectoryAdapter(RvizAdapter):
    def __init__(self, name : str, in_topic : str, out_topic : str):
        super().__init__(name, self.trajectory_adapter, in_topic, Point, out_topic, Marker)

    def trajectory_adapter(self, msg : Route) -> Marker:

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

        for i in range(msg.route): # type:ignore
            v = msg.route[i].speed  # type:ignore
            color_p : ColorRGBA = ColorRGBA()
            color_p.r = float(1 - (float(v) / m))
            color_p.g = (float(v) / m)
            color_p.b = float(0)
            color_p.a = 1.0
            marker.colors.append(color_p) # type:ignore

            viz_p : Point = msg.route[i] # type:ignore
            marker.points.append(viz_p) # type:ignore

        return marker
