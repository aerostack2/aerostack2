
#include "goto_plugin_base/goto_base.hpp"

class As2GotoBaseTest : public goto_base::GotoBase {
public:
  As2GotoBaseTest(){};
  bool on_deactivate(const std::shared_ptr<std::string> &message) { return false; };
  bool on_pause(const std::shared_ptr<std::string> &message) { return false; };
  bool on_resume(const std::shared_ptr<std::string> &message) { return false; };
  as2_behavior::ExecutionStatus own_run() { return as2_behavior::ExecutionStatus::SUCCESS; };
  void own_execution_end(const as2_behavior::ExecutionStatus &state){};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<As2GotoBaseTest>();
  rclcpp::shutdown();
  return 0;
}