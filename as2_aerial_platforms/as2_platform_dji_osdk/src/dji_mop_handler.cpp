#include "dji_mop_handler.hpp"

void DJIMopHandler::downlinkCB(const std_msgs::msg::String::SharedPtr msg) {
  uint8_t *sendBuf;
  sendBuf = (uint8_t *)OsdkOsal_Malloc(RELIABLE_SEND_ONCE_BUFFER_SIZE);

  if (sendBuf == NULL) {
    RCLCPP_ERROR(this->node_ptr_->get_logger(), "Osdk_malloc sendbuffer error");
  }

  MopPipeline::DataPackType writePack = {(uint8_t *)sendBuf,
                                         RELIABLE_SEND_ONCE_BUFFER_SIZE};

  // TODO: good?
  sendBuf = const_cast<uint8_t *>(
      reinterpret_cast<const uint8_t *>(msg->data.c_str() + '\r'));
  writePack.length = RELIABLE_SEND_ONCE_BUFFER_SIZE;
  DJI::OSDK::MOP::MopErrCode ret =
      pipeline_->sendData(writePack, &writePack.length);
};

void DJIMopHandler::keepAliveCB(const std_msgs::msg::String::SharedPtr msg) {
  status_ = msg->data + '\r';
};

void DJIMopHandler::publishUplink(const MopPipeline::DataPackType *dataPack) {
  std::string result(reinterpret_cast<const char *>(dataPack->data),
                     dataPack->length);
  std_msgs::msg::String msg = std_msgs::msg::String();
  msg.data = result;

  uplink_pub_->publish(msg);
}

void DJIMopHandler::mopCommunicationFnc(int id) {
  DJI::OSDK::MOP::PipelineID _id(id);
  DJI::OSDK::MOP::PipelineType type = DJI::OSDK::MOP::PipelineType::RELIABLE;
  MopErrCode ret = vehicle_ptr_->mopServer->accept(_id, type, pipeline);

  while (true) {
  }
};

std::string DJIMopHandler::bytesToString(const uint8_t *data, size_t len) {
  std::string result(reinterpret_cast<const char *>(data), len);
  return result;
}
