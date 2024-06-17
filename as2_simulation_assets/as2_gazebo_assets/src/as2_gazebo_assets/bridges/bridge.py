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
"""bridge.py."""

from dataclasses import dataclass
from enum import Enum


class BridgeDirection(Enum):
    """Enum for bridge directions."""

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
    """Bridge Gz<->Ros."""

    gz_topic: str
    ros_topic: str
    gz_type: str
    ros_type: str
    direction: BridgeDirection

    def argument(self):
        """Return argument for ros_gz_bridge."""
        out = f'{self.gz_topic}@{self.ros_type}{DIRECTION_SYMS[self.direction]}{self.gz_type}'
        return out

    def remapping(self):
        """Return remapping for ros_gz_bridge."""
        return (self.gz_topic, self.ros_topic)
