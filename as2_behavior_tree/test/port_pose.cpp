#include "as2_behavior_tree/port_specialization.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"

using namespace BT;

class CalculateGoal : public SyncActionNode {
public:
  CalculateGoal(const std::string &name, const NodeConfiguration &config)
      : SyncActionNode(name, config) {}

  NodeStatus tick() override {
    geometry_msgs::msg::Pose mygoal;
    mygoal.position.x = 1.0;
    mygoal.position.y = 2.0;
    mygoal.position.z = 3.0;
    setOutput("goal", mygoal);
    return NodeStatus::SUCCESS;
  }
  static PortsList providedPorts() {
    return {OutputPort<geometry_msgs::msg::Pose>("goal")};
  }
};

class PrintTarget : public SyncActionNode {
public:
  PrintTarget(const std::string &name, const NodeConfiguration &config)
      : SyncActionNode(name, config) {}

  NodeStatus tick() override {
    auto res = getInput<geometry_msgs::msg::Pose>("target");
    if (!res) {
      throw RuntimeError("error reading port [target]:", res.error());
    }
    geometry_msgs::msg::Pose goal = res.value();
    printf("Target positions: [ %.1f, %.1f, %.1f ]\n", goal.position.x,
           goal.position.y, goal.position.z);
    return NodeStatus::SUCCESS;
  }

  static PortsList providedPorts() {
    // Optionally, a port can have a human readable description
    const char *description = "Simply print the target on console...";
    return {InputPort<geometry_msgs::msg::Pose>("target", description)};
  }
};

//----------------------------------------------------------------

/** The tree is a Sequence of 4 actions

*  1) Store a value of Position2D in the entry "GoalPosition"
*     using the action CalculateGoal.
*
*  2) Call PrintTarget. The input "target" will be read from the Blackboard
*     entry "GoalPosition".
*
*  3) Use the built-in action SetBlackboard to write the key "OtherGoal".
*     A conversion from string to Position2D will be done under the hood.
*
*  4) Call PrintTarget. The input "goal" will be read from the Blackboard
*     entry "OtherGoal".
*/

// clang-format off
static const char* xml_text = R"(

 <root main_tree_to_execute = "MainTree" >
     <BehaviorTree ID="MainTree">
        <Sequence name="root">
            <CalculateGoal   goal="{GoalPosition}" />
            <PrintTarget     target="{GoalPosition}" />
            <SetBlackboard   output_key="OtherGoal" value="-1;-2;-3" />
            <PrintTarget     target="{OtherGoal}" />
        </Sequence>
     </BehaviorTree>
 </root>
 )";

// clang-format on

#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"

int main() {
  using namespace BT;

  BehaviorTreeFactory factory;
  factory.registerNodeType<CalculateGoal>("CalculateGoal");
  factory.registerNodeType<PrintTarget>("PrintTarget");

  auto tree = factory.createTreeFromText(xml_text);
  StdCoutLogger logger_cout(tree);
  tree.tickRoot();

  /* Expected output:
   *
      Target positions: [ 1.0, 2.0, 3.0 ]
      Target positions: [ -1.0, -2.0, -3.0 ]
  */
  return 0;
}
