.. _as2_behaviors_collision_avoidance:

Collision Avoidance Behavior (``as2_behaviors_collision_avoidance``)
====================================================================

.. toctree::
   :maxdepth: 2
   :hidden:

   self

Overview
--------

``as2_behaviors_collision_avoidance`` is an Aerostack2 behavior that
implements the *consensus* collective model from the CORESENSE Collective
Awareness architecture.  Before commanding the drone to fly a path it
acquires a distributed path lock from all known peers through the
:ref:`as2_ca` (CA Gateway); motion begins only when every peer has granted
the lock.

The behavior is designed for use in multi-UAV inspection scenarios where
drones share a confined airspace.  It integrates with the standard Aerostack2
``GoToBehavior`` for the motion phase and with the :ref:`state_interface`
for self-model queries.

Plugin architecture
~~~~~~~~~~~~~~~~~~~~

The path-conflict detection and lock-arbitration algorithm is encapsulated in
a :cpp:class:`collision_avoidance_base::CollisionAvoidanceBase` plugin loaded
via ``pluginlib``.  The default plugin is
``pairwise_path_lock_plugin::Plugin``, which implements a pairwise
request/grant/release protocol and detects conflicts by computing the minimum
polyline distance between two paths using :cpp:func:`path_geometry::min_polyline_distance`.

Protocol
--------

Modelet types
~~~~~~~~~~~~~

Three modelet types flow through the CA Gateway during a lock cycle:

``CAPathLockRequest``
  Sent to all known peers when this drone wants to fly a path.  Carries the
  path (as a sequence of :cpp:class:`as2_msgs::msg::PoseStampedWithID`), a
  safety distance, and a monotonically increasing request ID.

``CAPathLockGrant``
  Sent by a peer to the requester when the peer approves the path.  A grant
  is either immediate (no conflict) or deferred (peer will grant after its
  own motion completes).

``CAPathLockRelease``
  Broadcast by the lock holder after its motion completes.  Unblocks any
  peers that were deferred.

Lifecycle phases
~~~~~~~~~~~~~~~~~

The ``on_run`` method of :cpp:class:`collision_avoidance_behavior::CollisionAvoidanceBehavior`
drives a four-phase state machine:

.. code-block:: text

   REQUESTING_LOCK
       │  (all peers grant)
       ▼
   HOLDING_LOCK
       │  (GoToWaypoint goal sent, or no motion requested)
       ▼
   EXECUTING_MOTION
       │  (GoToWaypoint succeeds/fails/aborts)
       ▼
   RELEASING_LOCK
       │  (plugin returns IDLE)
       ▼
   SUCCESS

Path prepending
~~~~~~~~~~~~~~~

On activation the behavior reads the drone's current pose from the
:ref:`state_interface` and prepends it to the caller-supplied path.  This
ensures the lock covers the full corridor from the drone's current position to
the first waypoint, preventing a peer from crossing that opening segment during
lock acquisition.

Pause and Resume
~~~~~~~~~~~~~~~~

On ``pause``:

1. The lock is released unconditionally so that peers can claim the path.
2. The in-progress ``GoToBehavior`` is paused via a synchronous service call.

On ``resume``:

1. The full stored path is re-submitted for lock acquisition.
2. Once the lock is re-acquired, ``GoToBehavior`` is resumed via a service call.

This guarantees that no flight corridor is left unguarded during a pause/resume
cycle.

Usage
-----

Launching the behavior
~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   from launch_ros.actions import Node

   collision_avoidance = Node(
       package='as2_behaviors_collision_avoidance',
       executable='as2_behaviors_collision_avoidance_node',
       namespace=drone_id,
       parameters=[{
           'plugin_name': 'pairwise_path_lock_plugin::Plugin',
           'safety_distance': 1.5,
       }],
       output='screen',
   )

Sending a goal from a mission script
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The action is typically wrapped by the ``as2_python_api``:

.. code-block:: python

   from as2_msgs.msg import PoseStampedWithID
   from geometry_msgs.msg import PoseStamped

   def make_waypoint(name, x, y, z):
       wp = PoseStampedWithID()
       wp.id = name
       wp.pose.header.frame_id = 'earth'
       wp.pose.pose.position.x = x
       wp.pose.pose.position.y = y
       wp.pose.pose.position.z = z
       return wp

   path = [
       make_waypoint('wp0', 0.0, 0.0, 2.0),
       make_waypoint('wp1', 5.0, 0.0, 2.0),
   ]

   goal_pose = PoseStamped()
   goal_pose.header.frame_id = 'earth'
   goal_pose.pose.position.x = 5.0
   goal_pose.pose.position.y = 0.0
   goal_pose.pose.position.z = 2.0

   drone.collision_avoidance.go_to(
       path=path,
       goal_pose=goal_pose,
       safety_distance=1.5,
       max_speed=1.0,
   )

Writing a custom plugin
~~~~~~~~~~~~~~~~~~~~~~~~

Subclass :cpp:class:`collision_avoidance_base::CollisionAvoidanceBase`:

.. code-block:: cpp

   #include "as2_behaviors_collision_avoidance/collision_avoidance_base.hpp"

   namespace my_ca
   {
   class Plugin : public collision_avoidance_base::CollisionAvoidanceBase
   {
   public:
     void initialize(
       rclcpp::Node * node,
       std::shared_ptr<as2_ca::CAGatewayClient> ca,
       const std::string & own_id) override
     {
       node_ = node;
       ca_client_ = ca;
       own_id_ = own_id;
     }

     void start_acquisition(
       const std::vector<as2_msgs::msg::PoseStampedWithID> & path,
       const std::vector<std::string> & peers,
       uint32_t req_id,
       double safety_distance) override
     {
       // Send CAPathLockRequest to each peer via ca_client_
     }

     void release() override
     {
       // Send CAPathLockRelease to deferred peers
     }

     void on_request(const as2_msgs::msg::CAPathLockRequest & req,
                     const std::string & sender) override {}

     void on_grant(const as2_msgs::msg::CAPathLockGrant & grant,
                   const std::string & sender) override {}

     void on_release(const as2_msgs::msg::CAPathLockRelease & rel,
                     const std::string & sender) override {}

     collision_avoidance_base::Status status() const override { return status_; }

   private:
     rclcpp::Node * node_;
     std::shared_ptr<as2_ca::CAGatewayClient> ca_client_;
     std::string own_id_;
     collision_avoidance_base::Status status_;
   };
   }  // namespace my_ca

   #include <pluginlib/class_list_macros.hpp>
   PLUGINLIB_EXPORT_CLASS(my_ca::Plugin,
                          collision_avoidance_base::CollisionAvoidanceBase)

ROS 2 interface
---------------

Action
~~~~~~

``CollisionAvoidanceBehavior`` (``as2_msgs/action/CollisionAvoidance``)

========== =============================== ========================================
Field      Type                            Description
========== =============================== ========================================
Goal       ``path[]``                      Path to lock (``PoseStampedWithID[]``)
Goal       ``goal_pose``                   Motion target (zero → lock only, no motion)
Goal       ``safety_distance`` (float)     Minimum separation distance (m)
Goal       ``max_speed`` (float)           GoToWaypoint speed (m/s)
Goal       ``plugin_name`` (string)        Override the loaded plugin at goal time
Feedback   ``state``, ``lock_held``        Plugin phase and lock status
Feedback   ``pending_peers[]``             Peers not yet granted
Feedback   ``conflicting_peers[]``         Peers whose path overlaps
Result     ``collision_avoidance_success`` Motion outcome
========== =============================== ========================================

Parameters
~~~~~~~~~~

================================ ========================== ===========================
Parameter                        Default                    Description
================================ ========================== ===========================
``plugin_name``                  ``pairwise_path_lock_plugin::Plugin``  Plugin to load
``safety_distance``              ``1.5``                    Default safety distance (m)
================================ ========================== ===========================

Path geometry utilities
-----------------------

The ``path_geometry`` namespace provides a utility used by the default plugin
to detect path conflicts:

.. doxygenfunction:: path_geometry::min_polyline_distance
   :project: as2_behaviors_collision_avoidance

API Reference
-------------

CollisionAvoidanceBehavior
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. doxygenclass:: collision_avoidance_behavior::CollisionAvoidanceBehavior
   :project: as2_behaviors_collision_avoidance
   :members:

CollisionAvoidanceBase
~~~~~~~~~~~~~~~~~~~~~~

.. doxygenclass:: collision_avoidance_base::CollisionAvoidanceBase
   :project: as2_behaviors_collision_avoidance
   :members:

Status
~~~~~~

.. doxygenstruct:: collision_avoidance_base::Status
   :project: as2_behaviors_collision_avoidance
   :members:
