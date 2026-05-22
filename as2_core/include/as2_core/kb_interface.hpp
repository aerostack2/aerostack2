// Copyright 2026 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*!*******************************************************************************************
 *  \file       kb_interface.hpp
 *  \brief      Knowledge base interface header file
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#ifndef AS2_CORE__KB_INTERFACE_HPP_
#define AS2_CORE__KB_INTERFACE_HPP_

#include <functional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "kb_msgs/srv/event.hpp"
#include "kb_msgs/srv/query.hpp"
#include "std_msgs/msg/string.hpp"

namespace as2
{

/**
 * @brief Interface for Aerostack2 behaviors to interact with the CORESENSE knowledge base.
 *
 * @c KBInterface connects any Aerostack2 behavior or mission script to the
 * @c knowledge_core RDF triple store.  It exposes three families of operations:
 *
 * - **Fact assertion / retraction**: publish a triple to @c kb/add_fact or
 *   @c kb/remove_fact using a fire-and-forget publisher (non-blocking).
 *
 * - **Reactive event handling**: subscribe to a KB event topic so that a
 *   callback is invoked each time the KB produces a matching binding.
 *
 * - **Synchronous queries**: call the @c kb/query service and return the
 *   variable-binding results; uses a dedicated background executor thread so
 *   that the call is safe from inside a ROS 2 callback (avoids the
 *   "node already added to an executor" re-entrancy crash).
 *
 * @note Construct one @c KBInterface per behavior node.  The object owns a
 *       background @c SingleThreadedExecutor thread that is cleaned up in the
 *       destructor.
 */
class KBInterface
{
public:
  /**
   * @brief Lightweight representation of an RDF triple (subject–predicate–object).
   *
   * Used as input to @ref add_fact, @ref remove_fact, @ref query_kb, and as
   * the argument type of event-handler callbacks.
   */
  struct Triple
  {
    std::string subject;   ///< RDF subject term
    std::string predicate; ///< RDF predicate (relation name)
    std::string object;    ///< RDF object term

    Triple(const std::string & subj, const std::string & pred, const std::string & obj)
    : subject(subj), predicate(pred), object(obj) {}

    bool operator==(const Triple & other) const
    {
      return subject == other.subject && predicate == other.predicate && object == other.object;
    }

    /** @brief Serialize to a space-separated string: @c "subject predicate object". */
    std::string to_string() const
    {
      return subject + " " + predicate + " " + object;
    }

    /** @brief Human-readable representation for logging. */
    std::string repr() const
    {
      return "Triple(subject='" + subject + "', predicate='" + predicate + "', object='" +
             object + "')";
    }
  };

  /**
   * @brief Construct the interface and connect to the KB topics and services.
   * @param node_ptr Node used to create publishers, subscriptions, and clients.
   *                 Must outlive this object.
   */
  explicit KBInterface(rclcpp::Node * node_ptr);
  ~KBInterface();

  /**
   * @brief Assert a fact into the knowledge base (non-blocking).
   *
   * Publishes the triple @c "subj pred obj" to @c kb/add_fact.
   *
   * @param subj RDF subject.
   * @param pred RDF predicate.
   * @param obj  RDF object.
   */
  void add_fact(const std::string & subj, const std::string & pred, const std::string & obj);

  /**
   * @brief Retract a fact from the knowledge base (non-blocking).
   *
   * Publishes the triple @c "subj pred obj" to @c kb/remove_fact.
   *
   * @param subj RDF subject.
   * @param pred RDF predicate.
   * @param obj  RDF object.
   */
  void remove_fact(const std::string & subj, const std::string & pred, const std::string & obj);

  /**
   * @brief Query the KB and return the first matching binding (blocking).
   *
   * Calls the @c kb/query service and returns the first result binding, or an
   * empty map if no match is found or the call times out.
   *
   * @param clauses   Conjunctive triple patterns; terms starting with @c ? are variables.
   * @param variables Variable names to project into the result.
   * @return First binding as a @c {variable_name → value} map, or @c {} on failure.
   */
  std::unordered_map<std::string, std::string> query_kb(
    const std::vector<Triple> & clauses, const std::vector<std::string> & variables) const;

  /**
   * @brief Query the KB and return all matching bindings (blocking).
   *
   * Calls the @c kb/query service and returns every result binding.  Safe to
   * call from inside a ROS 2 callback because the underlying service client
   * runs on a dedicated background executor.
   *
   * @param clauses   Conjunctive triple patterns; terms starting with @c ? are variables.
   * @param variables Variable names to project into the result.
   * @return All bindings as a list of @c {variable_name → value} maps.
   *         Returns @c {} on timeout (5 s) or service error.
   */
  std::vector<std::unordered_map<std::string, std::string>> query_kb_all(
    const std::vector<Triple> & clauses, const std::vector<std::string> & variables) const;

  /**
   * @brief Subscribe to a KB event topic and invoke @p handler on each trigger.
   *
   * @param event_name Full ROS 2 topic name of the KB event to monitor.
   * @param handler    Callback invoked with the triggering triple on each event.
   */
  void register_event_handler(
    const std::string & event_name,
    std::function<void(const Triple &)> handler);

private:
  rclcpp::Node * node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr add_fact_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr remove_fact_pub_;
  rclcpp::Client<kb_msgs::srv::Event>::SharedPtr event_client_;
  rclcpp::Client<kb_msgs::srv::Query>::SharedPtr query_client_;
  std::unordered_map<std::string,
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> event_handlers_;

  // Dedicated node + executor for service clients so that query_kb_all can
  // wait on the future without re-entering the caller's executor.
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::executors::SingleThreadedExecutor client_executor_;
  std::thread spin_thread_;
};

}  // namespace as2

#endif  // AS2_CORE__KB_INTERFACE_HPP_
