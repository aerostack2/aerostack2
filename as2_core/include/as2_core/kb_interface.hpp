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

class KBInterface
{
public:
  struct Triple
  {
    std::string subject;
    std::string predicate;
    std::string object;

    Triple(const std::string & subj, const std::string & pred, const std::string & obj)
    : subject(subj), predicate(pred), object(obj) {}

    bool operator==(const Triple & other) const
    {
      return subject == other.subject && predicate == other.predicate && object == other.object;
    }

    std::string to_string() const
    {
      return subject + " " + predicate + " " + object;
    }

    std::string repr() const
    {
      return "Triple(subject='" + subject + "', predicate='" + predicate + "', object='" +
             object + "')";
    }
  };

  explicit KBInterface(rclcpp::Node * node_ptr);
  ~KBInterface();

  void add_fact(const std::string & subj, const std::string & pred, const std::string & obj);
  void remove_fact(const std::string & subj, const std::string & pred, const std::string & obj);
  std::unordered_map<std::string, std::string> query_kb(
    const std::vector<Triple> & clauses, const std::vector<std::string> & variables) const;

  std::vector<std::unordered_map<std::string, std::string>> query_kb_all(
    const std::vector<Triple> & clauses, const std::vector<std::string> & variables) const;

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
