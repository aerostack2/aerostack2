
#include "follow_path_plugin_base/follow_path_base.hpp"

class As2FollowPathBaseTest : public follow_path_base::FollowPathBase {
public:
  As2FollowPathBaseTest(){};
  bool own_activate(as2_msgs::action::FollowPath::Goal &goal) override { return true; };
  bool own_modify(as2_msgs::action::FollowPath::Goal &goal) override { return true; };
  bool own_deactivate(const std::shared_ptr<std::string> &message) override { return true; };
  bool own_pause(const std::shared_ptr<std::string> &message) override { return true; };
  bool own_resume(const std::shared_ptr<std::string> &message) override { return true; };
  void own_execution_end(const as2_behavior::ExecutionStatus &state) override{};
  as2_behavior::ExecutionStatus own_run() override {
    return as2_behavior::ExecutionStatus::SUCCESS;
  };
  Eigen::Vector3d getTargetPosition() override { return Eigen::Vector3d::Zero(); };
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<As2FollowPathBaseTest>();
  rclcpp::shutdown();
  return 0;
}