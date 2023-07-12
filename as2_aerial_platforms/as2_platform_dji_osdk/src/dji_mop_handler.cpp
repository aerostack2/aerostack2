#include "dji_mop_handler.hpp"

void DJIMopHandler::keepAliveCB(const std_msgs::msg::String::SharedPtr msg) {
  status_ = msg->data + '\r';
}

void DJIMopHandler::downlinkCB(const std_msgs::msg::String::SharedPtr msg) {
  if (!connected_) {
    return;
  }
  queue_mtx_.lock();
  std::string data = msg->data + '\r';
  msg_queue_.push(data);
  queue_mtx_.unlock();
}

// TODO: max retries change to use parameter
bool DJIMopHandler::send(int max_retries) {
  DJI::OSDK::MOP::MopErrCode ret;

  // Always appending status to send
  std::string data = status_;

  queue_mtx_.lock();
  if (msg_queue_.size() > 0) {
    // Appending msg from queue
    data.append(msg_queue_.front());
    msg_queue_.pop();
  }
  queue_mtx_.unlock();

  writePack_.length = strlen(data.c_str());
  memcpy(sendBuf_, data.c_str(), writePack_.length);

  for (int retry = 0; retry < max_retries; retry++) {
    ret = pipeline_->sendData(writePack_, &writePack_.length);
    switch (ret) {
      case DJI::OSDK::MOP::MopErrCode::MOP_PASSED:
        RCLCPP_INFO(node_ptr_->get_logger(), "[Keep Alive] Send %d bytes: %s",
                    writePack_.length, data.c_str());
        return true;
      case DJI::OSDK::MOP::MopErrCode::MOP_CONNECTIONCLOSE:
        return false;
      default:
        RCLCPP_INFO(node_ptr_->get_logger(),
                    "[SEND FAILED] MOP Code %d. Tried to send %d bytes", ret,
                    writePack_.length);
    }
  }
  return false;
}

// TODO: NOT USED
void DJIMopHandler::publishUplink(const MopPipeline::DataPackType *dataPack) {
  if (dataPack->length !=
      RELIABLE_RECV_ONCE_BUFFER_SIZE) {  // check this condition
    std_msgs::msg::String msg = std_msgs::msg::String();
    msg.data = bytesToString(dataPack->data, dataPack->length);

    uplink_pub_->publish(msg);
  }
}

bool DJIMopHandler::getReady() {
  recvBuf_ = (uint8_t *)OsdkOsal_Malloc(RELIABLE_RECV_ONCE_BUFFER_SIZE);
  if (recvBuf_ == NULL) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Osdk_malloc recvbuffer error");
    return false;
  }
  readPack_ = {(uint8_t *)recvBuf_, RELIABLE_RECV_ONCE_BUFFER_SIZE};

  sendBuf_ = (uint8_t *)OsdkOsal_Malloc(RELIABLE_SEND_ONCE_BUFFER_SIZE);
  if (sendBuf_ == NULL) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Osdk_malloc sendbuffer error");
    return false;
  }
  writePack_ = {(uint8_t *)sendBuf_, RELIABLE_SEND_ONCE_BUFFER_SIZE};

  return true;
}

void DJIMopHandler::mopCommunicationFnc(int id) {
  vehicle_ptr_->initMopServer();

  DJI::OSDK::MOP::PipelineID _id(id);
  DJI::OSDK::MOP::PipelineType type = DJI::OSDK::MOP::PipelineType::RELIABLE;
  DJI::OSDK::MOP::MopErrCode ret = DJI::OSDK::MOP::MopErrCode::MOP_NOTREADY;
  ret = vehicle_ptr_->mopServer->accept(_id, type, pipeline_);
  if (ret != DJI::OSDK::MOP::MOP_PASSED) {
    RCLCPP_INFO(node_ptr_->get_logger(), "Accept MOP err code: %d", ret);
    return;
  }

  RCLCPP_INFO(node_ptr_->get_logger(), "New client accepted");
  if (!getReady()) return;
  connected_ = true;

  while (true) {
    send(3);

    // Read
    memset(recvBuf_, 0, RELIABLE_RECV_ONCE_BUFFER_SIZE);
    readPack_.length = RELIABLE_RECV_ONCE_BUFFER_SIZE;
    ret = pipeline_->recvData(readPack_, &readPack_.length);

    // Parse reading
    size_t len = strlen(status_.c_str());

    RCLCPP_INFO(node_ptr_->get_logger(), "[Read] MOP Code %d. Bytes read %d",
                ret, readPack_.length);

    if (readPack_.length !=
        RELIABLE_RECV_ONCE_BUFFER_SIZE) {  // check this condition
      std_msgs::msg::String msg = std_msgs::msg::String();
      std::string msgs_to_send;
      msgs_to_send = bytesToString(readPack_.data, readPack_.length);
      if (missed_msg_ != "") {
        msgs_to_send = missed_msg_ + msgs_to_send;
        missed_msg_ = "";
      }
      auto [msg_parts, missed_msg_] = checkString(msgs_to_send, '\r');
      missed_msg_ = missed_msg_;
      for (const std::string &msg_ : msg_parts) {
        msg.data = msg_;
        RCLCPP_INFO(node_ptr_->get_logger(), "UPLINK MSG: %s\n", msg_.c_str());
        uplink_pub_->publish(msg);
      }
    }
    // publishUplink(&readPack);  // get whole uplink operation in the mutex

    if (ret == DJI::OSDK::MOP::MOP_CONNECTIONCLOSE) {
      close();
      break;
    }

    OsdkOsal_TaskSleepMs(1000);
    // do sleep
  }
  RCLCPP_ERROR(node_ptr_->get_logger(), "Connection closed. Exiting..");
}

void DJIMopHandler::close() {
  connected_ = false;
  closed_ = true;

  std::queue<std::string> empty;
  std::swap(msg_queue_, empty);
  missed_msg_ = "";

  vehicle_ptr_->mopServer->close(pipeline_->getId());
  pipeline_->~MopPipeline();
  pipeline_ = NULL;
  vehicle_ptr_->mopServer->~MopServer();
}

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
