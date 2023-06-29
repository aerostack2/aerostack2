#include "dji_mop_handler.hpp"

void DJIMopHandler::downlinkCB(const std_msgs::msg::String::SharedPtr msg) {
  if (!connected_) {
    return;
  }
  // TODO: check concurrency on pipeline_

  std::string data = msg->data + '\r';
  memcpy(downlinkBuf, data.c_str(), strlen(data.c_str()));
  downlinkPack.length = strlen(data.c_str());
  DJI::OSDK::MOP::MopErrCode ret =
      pipeline_->sendData(downlinkPack, &downlinkPack.length);
  // TODO: parse return MopCode
  RCLCPP_INFO(node_ptr_->get_logger(), "Downlink send: %d", ret);
};

void DJIMopHandler::keepAliveCB(const std_msgs::msg::String::SharedPtr msg) {
  status_ = msg->data + '\r';
};

void DJIMopHandler::publishUplink(const MopPipeline::DataPackType *dataPack) {
  std_msgs::msg::String msg = std_msgs::msg::String();
  msg.data = bytesToString(dataPack->data, dataPack->length);

  uplink_pub_->publish(msg);
}

void DJIMopHandler::mopCommunicationFnc(int id) {
  DJI::OSDK::MOP::PipelineID _id(id);
  DJI::OSDK::MOP::PipelineType type = DJI::OSDK::MOP::PipelineType::RELIABLE;
  DJI::OSDK::MOP::MopErrCode ret =
      vehicle_ptr_->mopServer->accept(_id, type, pipeline_);
  connected_ = true;

  recvBuf = (uint8_t *)OsdkOsal_Malloc(RELIABLE_RECV_ONCE_BUFFER_SIZE);
  if (recvBuf == NULL) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Osdk_malloc recvbuffer error");
    return;
  }
  readPack = {(uint8_t *)recvBuf, RELIABLE_RECV_ONCE_BUFFER_SIZE};

  sendBuf = (uint8_t *)OsdkOsal_Malloc(RELIABLE_SEND_ONCE_BUFFER_SIZE);
  if (sendBuf == NULL) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Osdk_malloc sendbuffer error");
    return;
  }
  writePack = {(uint8_t *)sendBuf, RELIABLE_SEND_ONCE_BUFFER_SIZE};

  downlinkBuf = (uint8_t *)OsdkOsal_Malloc(RELIABLE_SEND_ONCE_BUFFER_SIZE);
  if (downlinkBuf == NULL) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Osdk_malloc downlink_buffer error");
    return;
  }
  downlinkPack = {(uint8_t *)downlinkBuf, RELIABLE_SEND_ONCE_BUFFER_SIZE};

  while (true) {
    DJI::OSDK::MOP::MopErrCode ret;

    writePack.length = strlen(status_.c_str()) + 1;
    size_t len = strlen(status_.c_str());
    memcpy(sendBuf, status_.c_str(), writePack.length);
    ret = pipeline_->sendData(writePack, &writePack.length);
    RCLCPP_INFO(node_ptr_->get_logger(), "Keep alive send: %d", ret);

    // Read
    memset(recvBuf, 0, RELIABLE_RECV_ONCE_BUFFER_SIZE);
    readPack.length = RELIABLE_RECV_ONCE_BUFFER_SIZE;
    ret = pipeline_->recvData(readPack, &readPack.length);
    RCLCPP_INFO(node_ptr_->get_logger(), "Read: %d", ret);
    publishUplink(&readPack);
  }
};

std::string DJIMopHandler::bytesToString(const uint8_t *data, size_t len) {
  std::string result(reinterpret_cast<const char *>(data), len);
  return result;
}
