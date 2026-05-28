"""Fleet manager node — drone discovery, status monitoring, and swarm configuration."""

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

import json
from typing import List

from as2_fleet_manager.swarm_configurator import SwarmConfigurator
from as2_msgs.msg import PoseWithID
from as2_python_api.fleet_monitor import FleetMonitor
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default


class FleetManagerNode(Node):
    """Discovers as2 drones, monitors their status, and coordinates swarm configuration.

    Published topics
    ----------------
    ~/drone_namespaces  (std_msgs/String):
        JSON array of currently discovered drone namespaces.
        Published on every status cycle and whenever a drone is added/removed.

    ~/fleet_status  (std_msgs/String):
        JSON dict keyed by namespace with per-drone status fields:
        connected, armed, offboard, state (int), position ([x,y,z]).

    Services
    --------
    ~/start_swarm  (std_srvs/Trigger):
        Sends a SwarmFlocking goal with all *connected* drones using the
        default formation type and parameters from ROS parameters.
    """

    def __init__(self) -> None:
        super().__init__('fleet_manager')

        # Parameters
        self.declare_parameter('scan_interval', 5.0)
        self.declare_parameter('drone_timeout', 10.0)
        self.declare_parameter('publish_interval', 2.0)
        self.declare_parameter('swarm_behavior_server', '/Swarm/SwarmFlockingBehavior')
        self.declare_parameter('default_formation', 'line')
        self.declare_parameter('formation_spacing', 1.0)
        self.declare_parameter('formation_radius', 2.0)

        scan_interval = self.get_parameter('scan_interval').value
        drone_timeout = self.get_parameter('drone_timeout').value
        publish_interval = self.get_parameter('publish_interval').value
        swarm_server = self.get_parameter('swarm_behavior_server').value

        # Fleet monitor (attached to this node — no separate node needed)
        self._monitor = FleetMonitor(
            node=self,
            scan_interval=scan_interval,
            drone_timeout=drone_timeout,
            on_drone_found=self._on_drone_found,
            on_drone_lost=self._on_drone_lost,
        )

        # Swarm configurator
        self._swarm = SwarmConfigurator(self, swarm_server)

        # Publishers
        self._ns_pub = self.create_publisher(
            String, '~/drone_namespaces', qos_profile_system_default)
        self._status_pub = self.create_publisher(
            String, '~/fleet_status', qos_profile_system_default)

        # Services
        self.create_service(Trigger, '~/start_swarm', self._handle_start_swarm)

        # Periodic status publish
        self.create_timer(publish_interval, self._publish_status)

        self.get_logger().info(
            f'Fleet Manager started '
            f'(scan_interval={scan_interval}s, drone_timeout={drone_timeout}s)')

    # ------------------------------------------------------------------
    # Discovery callbacks
    # ------------------------------------------------------------------

    def _on_drone_found(self, namespace: str) -> None:
        self.get_logger().info(f'[Fleet] Drone registered: {namespace}')
        self._publish_namespaces()

    def _on_drone_lost(self, namespace: str) -> None:
        self.get_logger().warning(f'[Fleet] Drone removed: {namespace}')
        self._publish_namespaces()

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------

    def _publish_namespaces(self) -> None:
        msg = String()
        msg.data = json.dumps(sorted(self._monitor.drones.keys()))
        self._ns_pub.publish(msg)

    def _publish_status(self) -> None:
        drones = self._monitor.drones
        status = {}
        for ns, entry in drones.items():
            info = entry.info.data
            status[ns] = {
                'connected': bool(info[0]),
                'armed': bool(info[1]),
                'offboard': bool(info[2]),
                'state': int(info[3]),
                'position': entry.position,
            }
        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)
        self._publish_namespaces()

    # ------------------------------------------------------------------
    # Service handlers
    # ------------------------------------------------------------------

    def _handle_start_swarm(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Start swarm with all currently connected drones using default formation."""
        connected = self._monitor.connected_namespaces
        if not connected:
            response.success = False
            response.message = 'No connected drones found'
            self.get_logger().warning('[Fleet] start_swarm: no connected drones')
            return response

        centroid = PoseStamped()
        centroid.header.frame_id = 'earth'
        centroid.header.stamp = self.get_clock().now().to_msg()

        formation_type = self.get_parameter('default_formation').value
        spacing = self.get_parameter('formation_spacing').value
        radius = self.get_parameter('formation_radius').value

        ok = self._swarm.send_with_default_formation(
            drone_namespaces=connected,
            virtual_centroid=centroid,
            formation_type=formation_type,
            spacing=spacing,
            radius=radius,
        )

        response.success = ok
        response.message = (
            f'Swarm started with {connected}' if ok
            else 'Failed to send swarm goal'
        )
        return response

    # ------------------------------------------------------------------
    # Public API (callable from other Python nodes / scripts)
    # ------------------------------------------------------------------

    def request_swarm(
        self,
        drone_namespaces: List[str],
        virtual_centroid: PoseStamped,
        formation: List[PoseWithID],
    ) -> bool:
        """Send SwarmFlocking goal after verifying drone connectivity.

        :param drone_namespaces: Namespaces to include in the swarm.
        :param virtual_centroid: Reference pose for the swarm centre.
        :param formation: Per-drone PoseWithID offsets.
        :return: True if the goal was sent.
        """
        drones = self._monitor.drones
        for ns in drone_namespaces:
            if ns not in drones:
                self.get_logger().error(f'[Fleet] Drone not in fleet: {ns}')
                return False
            if not drones[ns].info.data[0]:  # connected
                self.get_logger().error(f'[Fleet] Drone not connected: {ns}')
                return False

        return self._swarm.send(drone_namespaces, virtual_centroid, formation)


def main():
    rclpy.init()
    node = FleetManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
