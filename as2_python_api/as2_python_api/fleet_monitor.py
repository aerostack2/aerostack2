"""Fleet monitor — discovers and tracks multiple as2 drones via ROS2 graph API."""

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

import threading
import time
from typing import Callable, Dict, List, Optional

from as2_msgs.msg import PlatformInfo
from as2_python_api.shared_data.platform_info_data import PlatformInfoData
from geometry_msgs.msg import PoseStamped

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default


_PLATFORM_INFO_SUFFIX = '/platform/info'


class DroneEntry:
    """Per-drone status container."""

    def __init__(self, namespace: str, timeout: float) -> None:
        self.namespace = namespace
        self.info = PlatformInfoData()
        self.position: List[float] = [0.0, 0.0, 0.0]
        self.last_seen: float = time.monotonic()
        self._timeout = timeout
        self._info_sub = None
        self._pose_sub = None

    @property
    def is_alive(self) -> bool:
        """True if a message was received within the timeout window."""
        return (time.monotonic() - self.last_seen) < self._timeout


class FleetMonitor:
    """Discovers and monitors as2 drones using the ROS2 graph API.

    Not a Node itself — attaches timers and subscriptions to the provided Node.
    Scans for ``*/platform/info`` topics periodically to detect new drones and
    tracks per-drone status via lightweight subscriptions.

    Drones that stop publishing within ``drone_timeout`` seconds are removed.

    Example usage::

        class MyNode(Node):
            def __init__(self):
                super().__init__('my_node')
                self.monitor = FleetMonitor(self, on_drone_found=self._on_found)

            def _on_found(self, ns):
                self.get_logger().info(f'Found: {ns}')
    """

    def __init__(
        self,
        node: Node,
        scan_interval: float = 5.0,
        drone_timeout: float = 10.0,
        on_drone_found: Optional[Callable[[str], None]] = None,
        on_drone_lost: Optional[Callable[[str], None]] = None,
    ) -> None:
        """
        Attach fleet monitoring to an existing ROS2 Node.

        :param node: The ROS2 node to attach timers/subscriptions to.
        :param scan_interval: Seconds between graph scans, defaults to 5.0
        :param drone_timeout: Seconds without messages before a drone is removed, defaults to 10.0
        :param on_drone_found: Callback(namespace) when a new drone is detected
        :param on_drone_lost: Callback(namespace) when a drone times out
        """
        self._node = node
        self._drone_timeout = drone_timeout
        self._on_drone_found = on_drone_found
        self._on_drone_lost = on_drone_lost
        self._lock = threading.Lock()
        self._drones: Dict[str, DroneEntry] = {}

        node.create_timer(scan_interval, self._scan_network)
        node.create_timer(drone_timeout / 2.0, self._check_timeouts)

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    @property
    def drones(self) -> Dict[str, DroneEntry]:
        """Thread-safe snapshot of all discovered drones."""
        with self._lock:
            return dict(self._drones)

    @property
    def connected_namespaces(self) -> List[str]:
        """Namespaces of drones where ``platform/info.connected`` is True."""
        with self._lock:
            return [ns for ns, e in self._drones.items() if e.info.data[0]]

    # ------------------------------------------------------------------
    # Internal — discovery
    # ------------------------------------------------------------------

    def _scan_network(self) -> None:
        """Scan ROS2 graph for new */platform/info topics."""
        topic_list = self._node.get_topic_names_and_types()
        found = {
            topic[: -len(_PLATFORM_INFO_SUFFIX)]
            for topic, _ in topic_list
            if topic.endswith(_PLATFORM_INFO_SUFFIX)
        }

        with self._lock:
            new_namespaces = found - set(self._drones.keys())

        for ns in new_namespaces:
            if ns:
                self._register_drone(ns)

    def _register_drone(self, namespace: str) -> None:
        entry = DroneEntry(namespace, self._drone_timeout)
        entry._info_sub = self._node.create_subscription(
            PlatformInfo,
            f'{namespace}/platform/info',
            lambda msg, ns=namespace: self._on_info(ns, msg),
            qos_profile_system_default,
        )
        entry._pose_sub = self._node.create_subscription(
            PoseStamped,
            f'{namespace}/self_localization/pose',
            lambda msg, ns=namespace: self._on_pose(ns, msg),
            qos_profile_sensor_data,
        )

        with self._lock:
            self._drones[namespace] = entry

        self._node.get_logger().info(f'[FleetMonitor] Drone discovered: {namespace}')
        if self._on_drone_found:
            self._on_drone_found(namespace)

    # ------------------------------------------------------------------
    # Internal — status callbacks
    # ------------------------------------------------------------------

    def _on_info(self, namespace: str, msg: PlatformInfo) -> None:
        with self._lock:
            if namespace not in self._drones:
                return
            entry = self._drones[namespace]

        entry.info.data = [
            msg.connected,
            msg.armed,
            msg.offboard,
            msg.status.state,
            msg.current_control_mode.yaw_mode,
            msg.current_control_mode.control_mode,
            msg.current_control_mode.reference_frame,
        ]
        entry.last_seen = time.monotonic()

    def _on_pose(self, namespace: str, msg: PoseStamped) -> None:
        with self._lock:
            if namespace not in self._drones:
                return
            entry = self._drones[namespace]

        entry.position = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ]
        entry.last_seen = time.monotonic()

    # ------------------------------------------------------------------
    # Internal — timeout handling
    # ------------------------------------------------------------------

    def _check_timeouts(self) -> None:
        with self._lock:
            lost = [ns for ns, e in self._drones.items() if not e.is_alive]

        for ns in lost:
            self._unregister_drone(ns)

    def _unregister_drone(self, namespace: str) -> None:
        with self._lock:
            if namespace not in self._drones:
                return
            entry = self._drones.pop(namespace)

        if entry._info_sub:
            self._node.destroy_subscription(entry._info_sub)
        if entry._pose_sub:
            self._node.destroy_subscription(entry._pose_sub)

        self._node.get_logger().warning(
            f'[FleetMonitor] Drone lost (timeout): {namespace}')
        if self._on_drone_lost:
            self._on_drone_lost(namespace)
