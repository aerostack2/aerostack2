// Copyright 2024 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include "as2_behavior_tree/port_specialization.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace BT
{
class CalculateGoal : public SyncActionNode
{
public:
  CalculateGoal(const std::string & name, const NodeConfiguration & config)
  : SyncActionNode(name, config) {}

  NodeStatus tick() override
  {
    geometry_msgs::msg::Pose mygoal;
    mygoal.position.x = 1.0;
    mygoal.position.y = 2.0;
    mygoal.position.z = 3.0;
    setOutput("goal", mygoal);
    return NodeStatus::SUCCESS;
  }
  static PortsList providedPorts()
  {
    return {OutputPort<geometry_msgs::msg::Pose>("goal")};
  }
};

class PrintTarget : public SyncActionNode
{
public:
  PrintTarget(const std::string & name, const NodeConfiguration & config)
  : SyncActionNode(name, config) {}

  NodeStatus tick() override
  {
    auto res = getInput<geometry_msgs::msg::Pose>("target");
    if (!res) {
      throw RuntimeError("error reading port [target]:", res.error());
    }
    geometry_msgs::msg::Pose goal = res.value();
    printf(
      "Target positions: [ %.1f, %.1f, %.1f ]\n", goal.position.x,
      goal.position.y, goal.position.z);
    return NodeStatus::SUCCESS;
  }

  static PortsList providedPorts()
  {
    // Optionally, a port can have a human readable description
    const char * description = "Simply print the target on console...";
    return {InputPort<geometry_msgs::msg::Pose>("target", description)};
  }
};
}  // end namespace BT
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
static const char * xml_text =
  R"(

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

int main()
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<BT::CalculateGoal>("CalculateGoal");
  factory.registerNodeType<BT::PrintTarget>("PrintTarget");

  auto tree = factory.createTreeFromText(xml_text);
  BT::StdCoutLogger logger_cout(tree);
  tree.tickRoot();

  /* Expected output:
   *
      Target positions: [ 1.0, 2.0, 3.0 ]
      Target positions: [ -1.0, -2.0, -3.0 ]
  */
  return 0;
}
