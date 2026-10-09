.. _state_interface:

State Interface (``StateInterface``)
=====================================

Overview
--------

``StateInterface`` is the *Agent Representation* component of the CORESENSE
Collective Awareness architecture.  It gives any Aerostack2 behavior a typed,
name-based read-only view of the drone's own state (pose, twist, battery, …)
without hard-coding topic names or message types in behavior code.

The class maintains a set of active ROS 2 subscriptions created at
configuration time.  Each subscription stores the last received message; the
behavior retrieves the current value by name via :cpp:func:`StateInterface::get_value`.

Design
------

Static type registry
~~~~~~~~~~~~~~~~~~~~~

``StateInterface`` uses a self-registration pattern to map topic-name strings
to typed subscription factories at compile time.  The
:c:macro:`REGISTER_STATE_ENTRY` macro, placed at namespace scope in a header or
translation unit, inserts one factory into a process-global registry before
``main`` runs:

.. code-block:: text

   REGISTER_STATE_ENTRY(NAME, TYPE)
         │
         ▼
   StateInterface::get_registry()["NAME"] = factory<TYPE>
         │
         ▼ (called by configure())
   subscription<TYPE>(node, NAME) → ActiveRegisterEntry<TYPE>

Because the registry is populated through static initializers, the set of
available state topics is determined entirely at link time.  Adding a new
topic requires only a new ``REGISTER_STATE_ENTRY`` invocation; no change to
``StateInterface`` itself is needed.

Built-in state topics
~~~~~~~~~~~~~~~~~~~~~

Three entries are registered by the header itself:

============================================================  =============================================
Topic name (key)                                              Message type
============================================================  =============================================
``as2_names::topics::self_localization::pose``                ``geometry_msgs::msg::PoseStamped``
``as2_names::topics::self_localization::twist``               ``geometry_msgs::msg::TwistStamped``
``as2_names::topics::sensor_measurements::battery``           ``sensor_msgs::msg::BatteryState``
============================================================  =============================================

Usage
-----

Configure and read pose
~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: cpp

   #include "as2_core/state_interface.hpp"
   #include "as2_core/names/topics.hpp"
   #include "geometry_msgs/msg/pose_stamped.hpp"

   StateInterface si;

   // In node constructor or on_configure():
   si.configure(this, {
     as2_names::topics::self_localization::pose,
     as2_names::topics::self_localization::twist,
   });

   // Anywhere after configure():
   auto pose = si.get_value<geometry_msgs::msg::PoseStamped>(
                   as2_names::topics::self_localization::pose);

Embedding in a behavior plugin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``StateInterface`` is embedded (not inherited) in both
:cpp:class:`AuctionBehaviorPluginBase` and
:cpp:class:`CollisionAvoidanceBehavior`.  Plugins that need drone state declare
a ``state_component`` parameter listing the topic keys to activate:

.. code-block:: cpp

   void MyPlugin::configure(rclcpp::Node * node)
   {
     std::vector<std::string> keys =
       node->get_parameter("state_component").as_string_array();
     state_interface_.configure(node, keys);
   }

   float MyPlugin::compute_cost() const
   {
     auto pose = state_interface_.get_value<geometry_msgs::msg::PoseStamped>(
                     as2_names::topics::self_localization::pose);
     // ... use pose.pose.position ...
   }

Extending with new state topics
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To make an additional topic available to any behavior in the process, add the
macro invocation to the package's header that is included everywhere:

.. code-block:: cpp

   #include "as2_core/state_interface.hpp"
   #include "my_msgs/msg/my_state.hpp"

   REGISTER_STATE_ENTRY("my_namespace/my_topic", my_msgs::msg::MyState)

No other change is required.  Any behavior that subsequently calls
``si.configure(node, {"my_namespace/my_topic"})`` will receive a subscription
to that topic.

API Reference
-------------

.. doxygenclass:: StateInterface
   :project: as2_core
   :members:
   :protected-members:

.. doxygenmacro:: REGISTER_STATE_ENTRY
   :project: as2_core
