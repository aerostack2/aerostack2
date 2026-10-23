.. _swarmlink:

SwarmLink Collective Awareness Middleware
============================================

.. toctree::
   :maxdepth: 2
   :hidden:

   self

Overview
--------

``swarmlink`` is the Collective Awareness (CA) middleware layer for Aerostack2.
It implements the *Communication Interface* and *Organisation Handling*
subsystems of the CORESENSE D7.3 Collective Awareness Module, providing
peer-to-peer modelet exchange between drone instances without a centralised
broker.

The package exposes two artifacts:

* **CA Gateway Node** (:cpp:class:`swarmlink::CA_Gateway`) — one ROS 2 node per
  drone that owns all inter-agent traffic: peer discovery, inbound
  demultiplexing, and outbound forwarding.

* **CA Gateway Client** (:cpp:class:`swarmlink::CAGatewayClient`) — a lightweight
  library embedded in behaviors that provides a fleet-size-agnostic
  ``register / forward`` interface without knowledge of peer addresses or
  transport details.

Design principles
~~~~~~~~~~~~~~~~~

**Separation of concerns between use and transport.**
Behaviors declare *what* messages to receive and *where* to send them;
the gateway node handles *how* delivery is performed.  Peer addresses, network
topology, and serialization are completely hidden from behavior code.

**Non-invasive integration.**
The gateway runs as a companion process alongside an existing Aerostack2 stack
and requires no modification to any existing component.  Behaviors that do not
need inter-agent coordination are unaffected.

Architecture
------------

Message flow
~~~~~~~~~~~~

The sequence below shows the three operations a behavior performs through the
CA Gateway:

.. code-block:: text

   Behavior          SL Client          SL Node          Remote Agent
      │                  │                  │                  │
      │─register(T, cb)──▶                  │                  │
      │                  │─subscribe(T)────▶│                  │
      │                  │                  │                  │
      │                  │                  │◀─[msg of type T]─│
      │                  │◀─dispatch(msg)───│                  │
      │◀─cb(msg)─────────│                  │                  │
      │                  │                  │                  │
      │─forward(T, msg)──▶                  │                  │
      │                  │─route(msg)──────▶│                  │
      │                  │                  │──[network]──────▶│

**Step 1 – register.**
A behavior calls :cpp:func:`swarmlink::CAGatewayClient::register_module` with a
message-type tag and a callback.  The client contacts the local gateway's
``register_module`` service; the gateway creates a typed publisher for that tag
and returns the local topic name.  The client subscribes to that topic; from
this point any peer message of type ``T`` invokes the callback.

**Step 2 – receive.**
When a peer sends a message, it arrives on the drone's own incoming topic
(``<ns>/gateway_in``).  The gateway looks up the registered local publisher for
that type and re-publishes the payload as a ``LocalGenericMessage`` containing
the sender's agent ID.

**Step 3 – forward.**
A behavior calls :cpp:func:`swarmlink::CAGatewayClient::forward_IA_msg` with a
typed message and a list of receiver namespaces.  The client serializes the
message and publishes it on ``<ns>/gateway_out``; the gateway picks it up and
publishes it to each peer's incoming topic.

Peer discovery and failure detection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The CA Gateway runs a 1 Hz timer that scans the ROS 2 topic graph for topics
ending in ``/gateway_in``.  Each newly discovered topic causes a publisher to
that peer to be created.  Publishers whose topic has no subscribers are pruned
automatically, giving implicit failure detection: a crashed drone disappears
from the reachable peer set within one discovery cycle without any explicit
heartbeat mechanism.

Usage
-----

Embedding the client in a behavior
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: cpp

   #include "swarmlink/ca_gateway_client.hpp"
   #include "my_pkg/msg/my_modelet.hpp"

   class MyBehavior : public as2_behavior::BehaviorServer<...>
   {
     swarmlink::CAGatewayClient ca_client_;

     MyBehavior() : ..., ca_client_(this)
     {
       ca_client_.register_module<my_pkg::msg::MyModelet>(
         "my_modelet",          // type tag — must match the sender's tag
         "my_behavior",         // module name (for logging)
         [this](const my_pkg::msg::MyModelet & msg, const std::string & sender) {
           handle_modelet(msg, sender);
         });
     }

     void send_to_peers()
     {
       my_pkg::msg::MyModelet out_msg;
       // ... fill out_msg ...
       auto peers = ca_client_.get_known_peers();
       ca_client_.forward_IA_msg(out_msg, "my_modelet", peers);
     }
   };

Launching the gateway node
~~~~~~~~~~~~~~~~~~~~~~~~~~

The gateway node is typically included in the drone's launch file:

.. code-block:: python

   from launch_ros.actions import Node

   ca_gateway = Node(
       package='swarmlink',
       executable='ca_gateway_node',
       namespace=drone_id,
       output='screen',
   )

Comparison with raw ROS 2 DDS
------------------------------

+----------------------+----------------------------------+------------------------------+
| Dimension            | CA Gateway                       | Raw ROS 2 DDS                |
+======================+==================================+==============================+
| Discovery overhead   | Registry resolved once at send   | Graph queried per message    |
|                      | time                             | type at runtime              |
+----------------------+----------------------------------+------------------------------+
| Naming coupling      | Typed registry; no topic names   | Per-message-type topic names |
|                      | in behavior code                 | hard-coded                   |
+----------------------+----------------------------------+------------------------------+
| Transport flexibility| Backend swappable (DDS, Zenoh,   | Tied to DDS                  |
|                      | MQTT, …)                         |                              |
+----------------------+----------------------------------+------------------------------+
| Fleet reconfiguration| Handled automatically by the     | Explicit publisher/subscriber|
|                      | gateway's discovery timer        | rewiring required            |
+----------------------+----------------------------------+------------------------------+

API Reference
-------------

CA Gateway Node
~~~~~~~~~~~~~~~

.. doxygenclass:: swarmlink::CA_Gateway
   :project: swarmlink
   :members:
   :protected-members:

CA Gateway Client
~~~~~~~~~~~~~~~~~

.. doxygenclass:: swarmlink::CAGatewayClient
   :project: swarmlink
   :members:
