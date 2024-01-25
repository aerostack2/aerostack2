"""
bridge.py
"""

from dataclasses import dataclass
from enum import Enum


class BridgeDirection(Enum):
    """
    Enum for bridge directions.
    """
    BIDIRECTIONAL = 0
    GZ_TO_ROS = 1
    ROS_TO_GZ = 2


DIRECTION_SYMS = {
    BridgeDirection.BIDIRECTIONAL: '@',
    BridgeDirection.GZ_TO_ROS: '[',
    BridgeDirection.ROS_TO_GZ: ']',
}


@dataclass
class Bridge:
    """
    Bridge Gz<->Ros
    """
    gz_topic: str
    ros_topic: str
    gz_type: str
    ros_type: str
    direction: BridgeDirection

    def argument(self):
        """
        Return argument for ros_gz_bridge
        """
        out = f'{self.gz_topic}@{self.ros_type}{DIRECTION_SYMS[self.direction]}{self.gz_type}'
        return out

    def remapping(self):
        """
        Return remapping for ros_gz_bridge
        """
        return (self.gz_topic, self.ros_topic)
