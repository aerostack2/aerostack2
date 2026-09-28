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
 *  \file       kb_interface.cpp
 *  \brief      Knowledge base interface implementation file
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/node.hpp"
#include "std_msgs/msg/string.hpp"
#include <nlohmann/json.hpp>
#include "as2_core/kb_interface.hpp"
#include "kb_msgs/srv/query.hpp"

using json = nlohmann::json;

namespace as2
{

KBInterface::KBInterface(rclcpp::Node * node)
{
  node_ = node;
  add_fact_pub_ = node->create_publisher<std_msgs::msg::String>("kb/add_fact", 10);
  remove_fact_pub_ = node->create_publisher<std_msgs::msg::String>("kb/remove_fact", 10);

  // Use a dedicated node for service clients so that query_kb_all can wait on
  // the future without re-entering the caller node's executor (which would
  // crash with "Node already added to an executor" when called from a callback).
  client_node_ = std::make_shared<rclcpp::Node>(
    std::string(node->get_name()) + "_kb_client",
    std::string(node->get_namespace()));
  event_client_ = client_node_->create_client<kb_msgs::srv::Event>("kb/events");
  query_client_ = client_node_->create_client<kb_msgs::srv::Query>("kb/query");

  client_executor_.add_node(client_node_);
  spin_thread_ = std::thread([this]() {client_executor_.spin();});
}

KBInterface::~KBInterface()
{
  client_executor_.cancel();
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
}

void KBInterface::add_fact(
  const std::string & subj, const std::string & pred,
  const std::string & obj)
{
  std_msgs::msg::String msg;
  msg.data = Triple(subj, pred, obj).to_string();
  // RCLCPP_INFO(node_->get_logger(), "Adding fact: %s", msg.data.c_str());
  add_fact_pub_->publish(msg);
}

void KBInterface::add_facts(const std::vector<Triple> & facts)
{
  if (facts.empty()) {
    return;
  }
  std::ostringstream oss;
  for (size_t i = 0; i < facts.size(); ++i) {
    if (i > 0) {
      oss << "\n";
    }
    oss << facts[i].to_string();
  }
  std_msgs::msg::String msg;
  msg.data = oss.str();
  add_fact_pub_->publish(msg);
}

void KBInterface::remove_fact(
  const std::string & subj, const std::string & pred,
  const std::string & obj)
{
  std_msgs::msg::String msg;
  msg.data = Triple(subj, pred, obj).to_string();
  RCLCPP_INFO(node_->get_logger(), "Removing fact: %s", msg.data.c_str());
  remove_fact_pub_->publish(msg);
}

void KBInterface::register_event_handler(
  const std::string & event_name,
  std::function<void(const Triple &)> handler)
{
  auto callback = [handler](const std_msgs::msg::String::SharedPtr msg) {
      std::istringstream iss(msg->data);
      std::string subj, pred, obj;
      iss >> subj >> pred >> obj;
      handler(Triple(subj, pred, obj));
    };

  auto subscription = node_->create_subscription<std_msgs::msg::String>(
    event_name, 10, callback);

  event_handlers_[event_name] = subscription;
}

std::unordered_map<std::string, std::string> KBInterface::query_kb(
  const std::vector<Triple> & clauses, const std::vector<std::string> & variables) const
{
  auto all = query_kb_all(clauses, variables);
  return all.empty() ? std::unordered_map<std::string, std::string>{} : all[0];
}

std::vector<std::unordered_map<std::string, std::string>> KBInterface::query_kb_all(
  const std::vector<Triple> & clauses, const std::vector<std::string> & variables) const
{
  auto request = std::make_shared<kb_msgs::srv::Query::Request>();
  request->vars = variables;
  std::vector<std::string> clause_strings;
  for (const auto & clause : clauses) {
    clause_strings.push_back(clause.to_string());
  }
  request->patterns = clause_strings;

  auto future = query_client_->async_send_request(request);

  // Wait for the response without touching the caller's executor.
  // client_executor_ (background thread) handles spinning client_node_.
  auto timeout = std::chrono::seconds(5);
  if (future.wait_for(timeout) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call /kb/query service (timeout)");
    return {};
  }

  auto response = future.get();
  if (!response->success) {
    RCLCPP_ERROR(node_->get_logger(), "Query failed: %s", response->error_msg.c_str());
    return {};
  }

  std::vector<std::unordered_map<std::string, std::string>> results;
  auto json_array = json::parse(response->json);
  if (json_array.is_array()) {
    for (auto & binding : json_array) {
      std::unordered_map<std::string, std::string> row;
      for (auto & [key, value] : binding.items()) {
        row[key] = value.get<std::string>();
      }
      results.push_back(std::move(row));
    }
  }

  RCLCPP_INFO(node_->get_logger(), "Query returned %zu result(s)", results.size());
  return results;
}

}  // namespace as2
