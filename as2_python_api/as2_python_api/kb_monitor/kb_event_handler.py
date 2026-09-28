# Copyright 2026 Universidad Politécnica de Madrid
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

"""KB event handler: query helper, context object, and ROS 2 monitor node."""

__authors__ = 'Guillermo GP-Lenza'
__copyright__ = 'Copyright (c) 2026 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import json
import threading
from typing import Callable

from as2_msgs.msg import MissionUpdate
from as2_python_api.kb_monitor.kb_params import EventHandlerParams
from as2_python_api.mission_interpreter.mission import InterpreterStatus
from geometry_msgs.msg import PoseStamped
from kb_msgs.srv import Event, Query
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_sensor_data
from rclpy.subscription import Subscription
from std_msgs.msg import String


class _QueryHelper:
    """Dedicated spin-thread for blocking KB query calls.

    Runs a :class:`SingleThreadedExecutor` in a daemon thread so that
    :meth:`query` can block safely from inside a subscription callback without
    conflicting with the main executor (adding a node to two executors at once
    raises ``RuntimeError`` in rclpy).
    """

    _counter: int = 0
    _counter_lock: threading.Lock = threading.Lock()

    def __init__(self, query_service: str) -> None:
        with _QueryHelper._counter_lock:
            idx = _QueryHelper._counter
            _QueryHelper._counter += 1

        self._node = rclpy.create_node(f'_kb_query_helper_{idx}')
        self._cli = self._node.create_client(Query, query_service)

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()

    def query(
        self,
        patterns: list[str],
        vars: list[str] | None = None,
        models: list[str] | None = None,
        timeout_sec: float = 5.0,
    ) -> list[dict]:
        """Call the KB query service synchronously and return the bindings.

        Parameters
        ----------
        patterns:
            RDF triple patterns with ``?``-prefixed variables,
            e.g. ``['?drone rdf:type Robot', '?drone has_battery ?level']``.
        vars:
            Variables to return; ``None`` returns all bound variables.
        models:
            KB models to search; ``None`` / ``[]`` means all models.
        timeout_sec:
            Maximum seconds to wait for a response.

        Returns
        -------
        list[dict]
            List of variable-binding dicts, e.g. ``[{'drone': 'drone0', ...}]``.
            Returns an empty list on timeout or error.
        """
        req = Query.Request()
        req.patterns = patterns
        req.vars = vars if vars is not None else []
        req.models = models if models is not None else []

        if not self._cli.service_is_ready():
            if not self._cli.wait_for_service(timeout_sec=timeout_sec):
                self._node.get_logger().error(
                    f'KB query service not available: "{self._cli.srv_name}"'
                )
                return []

        future = self._cli.call_async(req)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())

        if not done.wait(timeout=timeout_sec):
            self._node.get_logger().error(
                f'KB query timed out after {timeout_sec}s (patterns: {patterns}).'
            )
            return []

        result: Query.Response = future.result()  # type: ignore
        if result is None:
            return []
        if not result.success:
            self._node.get_logger().error(f'KB query failed: {result.error_msg}')
            return []

        return json.loads(result.json)

    def destroy(self) -> None:
        """Shut down the dedicated executor and destroy the helper node."""
        self._executor.shutdown(timeout_sec=1.0)
        self._node.destroy_node()


class KBHandlerContext:
    """Provides ROS 2 helpers to KB event handler functions.

    An instance is created once per drone namespace and passed as the second
    argument to every handler function registered for that drone.

    Attributes
    ----------
    drone_namespace : str
        Namespace of the drone this context belongs to (e.g. ``"drone0"``).
    mission_status : InterpreterStatus | None
        Most recent mission interpreter status received on
        ``/{drone_namespace}/mission_status``.  Updated automatically;
        ``None`` until the first message arrives.
    pose : PoseStamped | None
        Most recent pose received on ``/{drone_namespace}/self_localization/pose``.
        Updated automatically; ``None`` until the first message arrives.
    """

    def __init__(
        self,
        drone_namespace: str,
        mission_update_pub: Publisher,
        add_fact_pub: Publisher,
        remove_fact_pub: Publisher,
        query_helper: _QueryHelper,
    ) -> None:
        self.drone_namespace = drone_namespace
        self.mission_status: InterpreterStatus | None = None
        self.pose: PoseStamped | None = None
        self._mission_update_pub = mission_update_pub
        self._add_fact_pub = add_fact_pub
        self._remove_fact_pub = remove_fact_pub
        self._query_helper = query_helper

    # ------------------------------------------------------------------
    # KB query
    # ------------------------------------------------------------------

    def query(
        self,
        patterns: list[str],
        vars: list[str] | None = None,
        models: list[str] | None = None,
        timeout_sec: float = 5.0,
    ) -> list[dict]:
        """Query the drone's knowledge base synchronously.

        Safe to call from inside an event callback.

        Parameters
        ----------
        patterns:
            RDF triple patterns with ``?``-prefixed variables.
        vars:
            Variables to return; ``None`` returns all bound variables.
        models:
            KB models to search; ``None`` / ``[]`` means all models.
        timeout_sec:
            Maximum seconds to wait.

        Returns
        -------
        list[dict]
            List of variable-binding dicts.  Empty list on timeout or error.
        """
        return self._query_helper.query(patterns, vars, models, timeout_sec)

    # ------------------------------------------------------------------
    # Mission interpreter
    # ------------------------------------------------------------------

    def publish_mission_update(self, msg: MissionUpdate) -> None:
        """Publish a :class:`MissionUpdate` to the drone's mission interpreter."""
        self._mission_update_pub.publish(msg)

    # ------------------------------------------------------------------
    # Knowledge base
    # ------------------------------------------------------------------

    def add_fact(self, fact: str) -> None:
        """Add an RDF triple to the drone's knowledge base.

        Parameters
        ----------
        fact:
            Space-separated RDF triple, e.g. ``'drone0 is_investigating target1'``.
        """
        msg = String()
        msg.data = fact
        self._add_fact_pub.publish(msg)

    def remove_fact(self, fact: str) -> None:
        """Remove an RDF triple from the drone's knowledge base."""
        msg = String()
        msg.data = fact
        self._remove_fact_pub.publish(msg)

    def get_active_namespaces(self) -> list[str]:
        """Return namespaces of all currently active drones on the ROS graph.

        Uses the same CA-gateway peer-discovery heuristic: any drone that is
        running exposes a ``/{ns}/gateway_in`` topic, so listing topics and
        filtering by that suffix gives the live fleet.
        """
        suffix = '/gateway_in'
        topics = self._query_helper._node.get_topic_names_and_types()
        return [
            name[1:-len(suffix)]
            for name, _ in topics
            if name.endswith(suffix) and len(name) > len(suffix) + 1
        ]


class KBMonitorNode(Node):
    """ROS 2 node that subscribes to KB events and dispatches to handler functions.

    Analogous to :class:`~as2_visualization.rviz_adapter.VizBridge`: it groups
    multiple handler registrations under a single node.

    Usage
    -----
    Create the node, call :meth:`register_handler` for every (drone, params) pair
    *before* starting the executor, then spin::

        monitor = KBMonitorNode('kb_monitor')
        monitor.register_handler('drone0', 'kb', params)
        rclpy.spin(monitor)
    """

    def __init__(self, name: str = 'kb_monitor') -> None:
        super().__init__(name)
        # Keep hard references so the subscriptions are not garbage-collected.
        self._subscriptions: list[Subscription] = []
        # Per-drone state (context + query helper + mission_status sub).
        self._drone_state: dict[str, dict] = {}

    def destroy_node(self) -> None:
        """Clean up query helper threads before destroying the node."""
        for state in self._drone_state.values():
            state['query_helper'].destroy()
        super().destroy_node()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def register_handler(
        self,
        drone_namespace: str,
        kb_namespace: str,
        params: EventHandlerParams,
    ) -> bool:
        """Register one KB event handler for a drone.

        Calls the knowledge_core ``/events`` service to obtain the event topic,
        then creates a ROS 2 subscription that invokes ``params.handler`` on
        every trigger.

        Parameters
        ----------
        drone_namespace:
            Drone namespace, e.g. ``"drone0"``.
        kb_namespace:
            Namespace of the KB node relative to the drone, e.g. ``"kb"``.
        params:
            Handler configuration produced by :class:`~kb_parsing.KBJSONParser`.

        Returns
        -------
        bool
            ``True`` if the subscription was successfully created.
        """
        ctx = self._get_or_create_context(drone_namespace, kb_namespace)

        events_srv = f'/{drone_namespace}/{kb_namespace}/events'
        cli = self.create_client(Event, events_srv)

        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                f'KB events service "{events_srv}" is not available; '
                f'skipping handler "{params.handler_id}".'
            )
            return False

        req = Event.Request()
        req.patterns = params.patterns
        req.one_shot = params.one_shot
        req.models = params.models

        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if not future.done() or future.result() is None:
            self.get_logger().error(
                f'Service call to "{events_srv}" timed out or failed; '
                f'skipping handler "{params.handler_id}".'
            )
            return False

        result: Event.Response = future.result()  # type: ignore
        if not result.id:
            self.get_logger().error(
                f'KB returned an empty event id for handler "{params.handler_id}"; skipping.'
            )
            return False

        event_topic: str = result.topic
        self.get_logger().info(
            f'Handler "{params.handler_id}" registered for drone "{drone_namespace}"; '
            f'listening on "{event_topic}".'
        )

        handler_func = params.handler

        def _callback(msg: String, _handler=handler_func, _ctx=ctx) -> None:
            try:
                bindings: list[dict] = json.loads(msg.data)
            except json.JSONDecodeError as exc:
                self.get_logger().error(
                    f'Failed to decode KB event payload: {exc}; raw: "{msg.data}"'
                )
                return
            try:
                _handler(bindings, _ctx)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(
                    f'Handler "{params.handler_id}" raised an exception: {exc}'
                )

        sub = self.create_subscription(String, event_topic, _callback, 10)
        self._subscriptions.append(sub)
        return True

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _get_or_create_context(self, drone_namespace: str, kb_namespace: str) -> KBHandlerContext:
        """Return a cached :class:`KBHandlerContext`, creating all resources if needed."""
        if drone_namespace in self._drone_state:
            return self._drone_state[drone_namespace]['context']

        mission_update_pub: Publisher = self.create_publisher(
            MissionUpdate, f'/{drone_namespace}/mission_update', 10
        )
        add_fact_pub: Publisher = self.create_publisher(
            String, f'/{drone_namespace}/{kb_namespace}/add_fact', 10
        )
        remove_fact_pub: Publisher = self.create_publisher(
            String, f'/{drone_namespace}/{kb_namespace}/remove_fact', 10
        )
        query_helper = _QueryHelper(f'/{drone_namespace}/{kb_namespace}/query')

        ctx = KBHandlerContext(
            drone_namespace,
            mission_update_pub,
            add_fact_pub,
            remove_fact_pub,
            query_helper,
        )

        def _mission_status_cb(msg: String, _ctx=ctx) -> None:
            try:
                _ctx.mission_status = InterpreterStatus.parse_raw(msg.data)
            except Exception as exc:
                self.get_logger().warn(f'Failed to decode mission status: {exc}')

        status_sub = self.create_subscription(
            String,
            f'/{drone_namespace}/mission_status',
            _mission_status_cb,
            10,
        )

        def _pose_cb(msg: PoseStamped, _ctx=ctx) -> None:
            _ctx.pose = msg

        pose_sub = self.create_subscription(
            PoseStamped,
            f'/{drone_namespace}/self_localization/pose',
            _pose_cb,
            qos_profile_sensor_data,
        )

        self._drone_state[drone_namespace] = {
            'context': ctx,
            'query_helper': query_helper,
            'mission_status_sub': status_sub,
            'pose_sub': pose_sub,
        }
        return ctx
