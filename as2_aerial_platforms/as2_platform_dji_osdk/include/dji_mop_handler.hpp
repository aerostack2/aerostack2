#ifndef DJI_MOP_HANDLER_HPP_
#define DJI_MOP_HANDLER_HPP_

#include <mutex>  // std::mutex
#include <rclcpp/rclcpp.hpp>
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
  DJI::OSDK::Vehicle* vehicle_ptr_;
  as2::Node* node_ptr_;
  DJI::OSDK::MopPipeline* pipeline_ = NULL;
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
        "/keep_alive", rclcpp::QoS(1),
        std::bind(&DJIMopHandler::keepAliveCB, this, std::placeholders::_1));

    vehicle_ptr_->initMopServer();

    mop_communication_th =
        std::thread(&DJIMopHandler::mopCommunicationFnc, this,
                    49152);  // This can be parametrized

    // CREATE THREAD-> accept_client, loop: sendData, readData
  };

  ~DJIMopHandler() {
    mop_communication_th.join();
    OsdkOsal_Free(recvBuf);
    OsdkOsal_Free(sendBuf);
    OsdkOsal_Free(downlinkBuf);
    pipeline_->~MopPipeline();
    vehicle_ptr_->mopServer->~MopServer();
  };

  void downlinkCB(const std_msgs::msg::String::SharedPtr msg);
  void keepAliveCB(const std_msgs::msg::String::SharedPtr msg);
  void mopCommunicationFnc(int id);

 private:
  std::string bytesToString(const uint8_t* data, size_t len);
  std::tuple<std::vector<std::string>, std::string> checkString(
      const std::string& input, char delimiter);
  void publishUplink(const MopPipeline::DataPackType* dataPack);

  std::mutex pipelin_mtx_;
  bool connected_ = false;
  std::string status_ = "{}\r";
  std::string missed_msg = "";
  std::thread mop_communication_th;
  uint8_t* recvBuf;
  uint8_t* sendBuf;
  uint8_t* downlinkBuf;
  MopPipeline::DataPackType readPack;
  MopPipeline::DataPackType writePack;
  MopPipeline::DataPackType downlinkPack;
};
#endif  // DJI_MOP_HANDLER_HPP_