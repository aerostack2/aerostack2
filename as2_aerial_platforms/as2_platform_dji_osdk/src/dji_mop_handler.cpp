#include "dji_mop_handler.hpp"

void DJIMopHandler::downlinkCB(std_msgs::msg::String){

};

void DJIMopHandler::keepAliveCB(std_msgs::msg::String){

};

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