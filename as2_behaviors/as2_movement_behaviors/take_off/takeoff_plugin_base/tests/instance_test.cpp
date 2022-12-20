
#include "takeoff_plugin_base/takeoff_base.hpp"

class As2TakeOffBaseTest : public takeoff_base::TakeOffBase {
public:
  As2TakeOffBaseTest(){};
  bool own_activate(as2_msgs::action::TakeOff::Goal &goal) override { return true; };
  bool own_modify(as2_msgs::action::TakeOff::Goal &goal) override { return true; };
  bool own_deactivate(const std::shared_ptr<std::string> &message) override { return true; };
  bool own_pause(const std::shared_ptr<std::string> &message) override { return true; };
  bool own_resume(const std::shared_ptr<std::string> &message) override { return true; };
  void own_execution_end(const as2_behavior::ExecutionStatus &state) override{};
  as2_behavior::ExecutionStatus own_run() override {
    return as2_behavior::ExecutionStatus::SUCCESS;
  };
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<As2TakeOffBaseTest>();
  rclcpp::shutdown();
  return 0;
}