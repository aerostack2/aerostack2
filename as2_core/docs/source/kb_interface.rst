.. _kb_interface:

Knowledge Base Interface — C++ (``as2::KBInterface``)
======================================================

Overview
--------

``as2::KBInterface`` connects any Aerostack2 behavior or mission script to the
CORESENSE `knowledge_core`_ RDF triple store.  It is the C++ layer of the
knowledge infrastructure; the complementary Python layer is described in the
:ref:`kb_monitor` documentation.

The interface exposes three families of operation:

* **Fact assertion / retraction** — publish or withdraw RDF triples from the
  knowledge base (non-blocking, publisher-based).
* **Reactive event handling** — register a callback invoked each time the KB
  produces a binding that matches a pattern.
* **Synchronous queries** — call the ``kb/query`` service and return
  variable-binding results; safe to call from inside a ROS 2 callback.

.. _knowledge_core: https://github.com/severin-lemaignan/knowledge_core

RDF triple model
~~~~~~~~~~~~~~~~

All facts are represented as subject–predicate–object triples:

.. code-block:: text

   drone0  assignedTo  panel_row_7
   drone0  auctionStatus  completed
   panel_row_7  xCoord  "12.5"

The :cpp:struct:`as2::KBInterface::Triple` struct is used throughout the API
to carry these three string components.

Threading model
~~~~~~~~~~~~~~~

``KBInterface`` runs a private :cpp:class:`rclcpp::executors::SingleThreadedExecutor`
on a dedicated background thread so that :cpp:func:`as2::KBInterface::query_kb_all`
can block on a service future without re-entering the caller node's executor.
This avoids the ``"Node already added to an executor"`` crash that would occur
if the same service client were spun from a subscription callback.

Usage
-----

Construct and assert facts
~~~~~~~~~~~~~~~~~~~~~~~~~~

``KBInterface`` is available inside any behavior that inherits from
:cpp:class:`as2_behavior::BehaviorServer` via the public ``kb_interface_``
member.  External nodes create their own instance:

.. code-block:: cpp

   #include "as2_core/kb_interface.hpp"

   as2::KBInterface kb(this);            // 'this' is an rclcpp::Node*

   kb.add_fact("drone0", "assignedTo", "panel_row_7");
   kb.add_fact("panel_row_7", "xCoord", "\"12.5\"");

   // Later, retract:
   kb.remove_fact("drone0", "assignedTo", "panel_row_7");

Query
~~~~~

.. code-block:: cpp

   using Triple = as2::KBInterface::Triple;

   // Find all panels assigned to drone0
   auto results = kb.query_kb_all(
     { Triple("?panel", "assignedTo", "drone0") },
     { "panel" }
   );

   for (const auto & row : results) {
     RCLCPP_INFO(get_logger(), "Assigned panel: %s", row.at("panel").c_str());
   }

   // Single-result convenience form
   auto row = kb.query_kb(
     { Triple("drone0", "auctionStatus", "?status") },
     { "status" }
   );
   if (!row.empty()) {
     std::string status = row.at("status");
   }

Reactive event handling
~~~~~~~~~~~~~~~~~~~~~~~~

Subscribe to a KB event topic obtained from the ``kb/events`` service and
register a callback to be invoked on each matching binding:

.. code-block:: cpp

   // The event topic name is obtained once from the knowledge_core events service.
   // Here we assume it has already been registered and the topic is known.
   kb.register_event_handler(
     "/drone0/kb/event/auction_completed",
     [this](const as2::KBInterface::Triple & triple) {
       RCLCPP_INFO(get_logger(), "Auction completed for %s", triple.subject.c_str());
       start_mission();
     });

ROS 2 topics and services
--------------------------

=================================  =========  =====================================
Topic / Service                    Direction  Purpose
=================================  =========  =====================================
``kb/add_fact``                    publish    Assert a triple
``kb/remove_fact``                 publish    Retract a triple
``kb/query``                       service    Return variable bindings for patterns
``kb/events``                      service    Register a reactive trigger
=================================  =========  =====================================

All names are relative to the node's namespace.

API Reference
-------------

.. doxygenstruct:: as2::KBInterface::Triple
   :project: as2_core
   :members:

.. doxygenclass:: as2::KBInterface
   :project: as2_core
   :members:
