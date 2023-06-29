#ifndef DJI_MOP_HANDLER_HPP_
#define DJI_MOP_HANDLER_HPP_

#include <thread>
#include "as2_core/names/topics.hpp"
#include "as2_core/node.hpp"
#include "dji_vehicle.hpp"
#include "std_msgs/msg/string.hpp"

#include "dji_linux_helpers.hpp"
#include "dji_vehicle.hpp"
#include "osdk_platform.h"
#include "osdkhal_linux.h"

#define RELIABLE_RECV_ONCE_BUFFER_SIZE (1024)
#define RELIABLE_SEND_ONCE_BUFFER_SIZE (1024)

class DJIMopHandler {
  DJI::OSDK::MopPipeline* pipeline_ = NULL;

  DJI::OSDK::Vehicle* vehicle_ptr_;
  as2::Node* node_ptr_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr uplink_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr downlink_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keep_alive_sub_;

  std::string status_ = "{}\r";

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

    std::thread mop_communication_th(&DJIMopHandler::mopCommunicationFnc, this,
                                     49152);  // This can be parametrized

    // CREATE THREAD-> accept_client, loop: sendData, readData
  };

  ~DJIMopHandler() {
    // mop_communication_th.join();
    pipeline->~MopPipeline();
    vehicle_ptr_->mopServer->~MopServer();
  };

  void downlinkCB(const std_msgs::msg::String::SharedPtr msg);
  void keepAliveCB(const std_msgs::msg::String::SharedPtr msg);
  void mopCommunicationFnc(int id);

 private:
  std::string bytesToString(const uint8_t* data, size_t len);
  void publishUplink(const MopPipeline::DataPackType* dataPack);

  DJI::OSDK::MopPipeline* pipeline = NULL;
  //   std::thread mop_communication_th;
  uint8_t* recvBuf;
  uint8_t* sendBuf;
  uint8_t* downlinkBuf;
  MopPipeline::DataPackType readPack;
  MopPipeline::DataPackType writePack;
  MopPipeline::DataPackType downlinkPack;
};
#endif  // DJI_MOP_HANDLER_HPP_