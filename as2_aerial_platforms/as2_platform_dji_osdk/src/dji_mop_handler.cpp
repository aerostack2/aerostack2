#include "dji_mop_handler.hpp"

void DJIMopHandler::downlinkCB(const std_msgs::msg::String::SharedPtr msg) {
  if (!connected_) {
    return;
  }
  // TODO: check concurrency on pipeline_

  std::string data = msg->data + '\r';
  memcpy(downlinkBuf, data.c_str(), strlen(data.c_str()));
  downlinkPack.length = strlen(data.c_str());

  pipelin_mtx_.lock();
  DJI::OSDK::MOP::MopErrCode ret =
      pipeline_->sendData(downlinkPack, &downlinkPack.length);
  OsdkOsal_TaskSleepMs(100);
  pipelin_mtx_.unlock();

  // TODO: parse return MopCode
  RCLCPP_INFO(node_ptr_->get_logger(), "Downlink send: %d", ret);
};

void DJIMopHandler::keepAliveCB(const std_msgs::msg::String::SharedPtr msg) {
  status_ = msg->data + '\r';
};

void DJIMopHandler::publishUplink(const MopPipeline::DataPackType *dataPack) {
  if (dataPack->length !=
      RELIABLE_RECV_ONCE_BUFFER_SIZE) {  // check this condition
    std_msgs::msg::String msg = std_msgs::msg::String();
    msg.data = bytesToString(dataPack->data, dataPack->length);

    uplink_pub_->publish(msg);
  }
}

void DJIMopHandler::mopCommunicationFnc(int id) {
  DJI::OSDK::MOP::PipelineID _id(id);
  DJI::OSDK::MOP::PipelineType type = DJI::OSDK::MOP::PipelineType::RELIABLE;
  DJI::OSDK::MOP::MopErrCode ret =
      vehicle_ptr_->mopServer->accept(_id, type, pipeline_);
  connected_ = true;
  RCLCPP_INFO(node_ptr_->get_logger(), "New client accepted: %d", ret);

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

    writePack.length = strlen(status_.c_str());

    memcpy(sendBuf, status_.c_str(), writePack.length);

    pipelin_mtx_.lock();
    ret = pipeline_->sendData(writePack, &writePack.length);
    OsdkOsal_TaskSleepMs(100);
    auto clk = node_ptr_->get_clock();
    RCLCPP_INFO_THROTTLE(node_ptr_->get_logger(), *clk, 5000,
                         "Keep alive send: %d", ret);

    // Read
    memset(recvBuf, 0, RELIABLE_RECV_ONCE_BUFFER_SIZE);
    readPack.length = RELIABLE_RECV_ONCE_BUFFER_SIZE;
    ret = pipeline_->recvData(readPack, &readPack.length);
    OsdkOsal_TaskSleepMs(100);
    // Parse reading
    size_t len = strlen(status_.c_str());

    RCLCPP_INFO_THROTTLE(node_ptr_->get_logger(), *clk, 5000,
                         "[Read] MOP Code %d. Bytes read %d", ret,
                         readPack.length);

    if (readPack.length !=
        RELIABLE_RECV_ONCE_BUFFER_SIZE) {  // check this condition
      std_msgs::msg::String msg = std_msgs::msg::String();
      std::string msgs_to_send;
      msgs_to_send = bytesToString(readPack.data, readPack.length);
      if (missed_msg != "") {
        msgs_to_send = missed_msg + msgs_to_send;
        missed_msg = "";
      }
      auto [msg_parts, missed_msg_] = checkString(msgs_to_send, '\r');
      missed_msg = missed_msg_;
      for (const std::string &msg_ : msg_parts) {
        msg.data = msg_;
        RCLCPP_INFO(node_ptr_->get_logger(), "UPLINK MSG: %s", msg_.c_str());
        uplink_pub_->publish(msg);
      }
    }
    // publishUplink(&readPack);  // get whole uplink operation in the mutex

    pipelin_mtx_.unlock();

    if (ret == DJI::OSDK::MOP::MOP_CONNECTIONCLOSE) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Connection closed. Exiting..");
      connected_ = false;
      break;
    } else if (ret == DJI::OSDK::MOP::MOP_TIMEOUT) {
      OsdkOsal_TaskSleepMs(1000);
    }
  }
};

std::tuple<std::vector<std::string>, std::string> DJIMopHandler::checkString(
    const std::string &input,
    char delimiter) {  // for /r delimiter
  std::vector<std::string> parts;
  std::stringstream ss(input);
  std::string part;
  std::string last_part = "";

  while (std::getline(ss, part, delimiter)) {
    parts.push_back(part);
  }
  if (parts.size() > 0 && input.back() != delimiter) {
    last_part = parts.back();
    RCLCPP_INFO(node_ptr_->get_logger(), "MISSED MSG: %s", last_part.c_str());
    parts.pop_back();
  }
  return {parts, last_part};
}

std::string DJIMopHandler::bytesToString(const uint8_t *data, size_t len) {
  std::string result(reinterpret_cast<const char *>(data), len);
  return result;
}
