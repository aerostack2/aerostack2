#include "dji_mop_handler.hpp"

void DJIMopHandler::downlinkCB(const std_msgs::msg::String::SharedPtr msg) {
  if (pipeline_ == NULL) {
    return;
  }

  // TODO: good?
  downlinkBuf = const_cast<uint8_t *>(
      reinterpret_cast<const uint8_t *>(msg->data.c_str() + '\r'));
  downlinkPack.length = RELIABLE_SEND_ONCE_BUFFER_SIZE;
  DJI::OSDK::MOP::MopErrCode ret =
      pipeline_->sendData(downlinkPack, &downlinkPack.length);
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
    writePack.length = strlen(status_.c_str()) + 1;
    size_t len = strlen(status_.c_str());
    memcpy(sendBuf, status_.c_str(), writePack.length);
    DJI::OSDK::MOP::MopErrCode ret;

    ret = pipeline_->sendData(writePack, &writePack.length);
    // Read
    memset(recvBuf, 0, RELIABLE_RECV_ONCE_BUFFER_SIZE);
    readPack.length = RELIABLE_RECV_ONCE_BUFFER_SIZE;
    ret = pipeline_->recvData(readPack, &readPack.length);
    publishUplink(&readPack);
  }
};

std::string DJIMopHandler::bytesToString(const uint8_t *data, size_t len) {
  std::string result(reinterpret_cast<const char *>(data), len);
  return result;
}
