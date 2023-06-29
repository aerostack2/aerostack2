#ifndef DJI_MOP_HANDLER_HPP_
#define DJI_MOP_HANDLER_HPP_

#include "as2_core/names/topics.hpp"
#include "as2_core/node.hpp"
#include "dji_vehicle.hpp"
#include "std_msgs/msg/string.hpp"

#include "dji_linux_helpers.hpp"
#include "dji_vehicle.hpp"
#include "osdk_platform.h"
#include "osdkhal_linux.h"

using namespace DJI::OSDK;

class DJIMopHandler {
  DJI::OSDK::Vehicle* vehicle_ptr_;
  as2::Node* node_ptr_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr uplink_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr downlink_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keep_alive_sub_;

 public:
  DJIMopHandler(DJI::OSDK::Vehicle* vehicle, as2::Node* node)
      : vehicle_ptr_(vehicle), node_ptr_(node) {
    uplink_pub_ = node_ptr_->create_publisher<std_msgs::msg::String>(
        "/uplink", as2_names::topics::global::qos);

    downlink_sub_ = node_ptr_->create_subscription<std_msgs::msg::String>(
        "/downlink", as2_names::topics::global::qos,
        std::bind(&DJIMopHandler::downlinkCB, this, std::placeholders::_1));

    keep_alive_sub_ = node_ptr_->create_subscription<std_msgs::msg::String>(
        "/keep_alive", as2_names::topics::global::qos,
        std::bind(&DJIMopHandler::keepAliveCB, this, std::placeholders::_1));

    vehicle_ptr_->initMopServer();

    // CREATE THREAD-> accept_client, loop: sendData, readData
  };
  void downlinkCB(std_msgs::msg::String);
  void keepAliveCB(std_msgs::msg::String);
};
#endif  // DJI_MOP_HANDLER_HPP_