// Copyright 2023 Universidad Politécnica de Madrid
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

/*!*******************************************************************************************
 *  \file       platform_state_machine.hpp
 *  \brief      Aerostack2 Platform State Machine Header file.
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 ********************************************************************************/

#ifndef AS2_CORE__PLATFORM_STATE_MACHINE_HPP_
#define AS2_CORE__PLATFORM_STATE_MACHINE_HPP_

#include <functional>
#include <string>
#include <vector>
#include <memory>

#include "as2_core/node.hpp"
#include "as2_msgs/msg/platform_state_machine_event.hpp"
#include "as2_msgs/msg/platform_status.hpp"
#include "as2_msgs/srv/set_platform_state_machine_event.hpp"

namespace as2
{
/**
 * @brief Event type.
 *
 */
using Event = as2_msgs::msg::PlatformStateMachineEvent;

/**
 * @brief Data Structure for defining the state machine transitions.
 */
struct StateMachineTransition
{
  std::string transition_name;
  int8_t from_state_id;
  int8_t transition_id;
  int8_t to_state_id;
};

/**
 * @brief This class implements the Platform State Machine,
 * which is in charge of handling the state of the platform using a FSM (Finite State Machine).
 * This state machine consist on 6 states:
 *   - DISARMED -> The platform is not armed.
 *   - LANDED -> The platform is armed and landed.
 *   - TAKING_OFF -> The platform is taking off.
 *   - FLYING -> The platform is on air.
 *   - LANDING -> The platform is landing.
 *   - EMERGENCY -> The platform is in emergency mode.
 *
 * The events that can trigger the state machine are:
 *   - ARM
 *   - DISARM
 *   - TAKE_OFF
 *   - TOOK_OFF
 *   - LAND
 *   - LANDED
 *   - EMERGENCY
 *  TODO(miferco97): add figure of the state machine
 *  \image html test.jpg
 */

class PlatformStateMachine
{
public:
  /**
   * @brief Constructor of the Platform State Machine.
   * @param node_ptr Pointer to an aerostack2 node.
   */
  explicit PlatformStateMachine(as2::Node * node);
  ~PlatformStateMachine();

  /**
   * @brief This function is in charge of handling the state machine.
   * @param event The event that triggers the state machine.
   * @return true If the event is valid in current State.
   */
  bool processEvent(const int8_t & event);
  /**
   * @brief This function is in charge of handling the state machine.
   * @param event The event that triggers the state machine.
   * @return true If the event is valid in current State.
   */
  bool processEvent(const Event & event);

  /**
   * @brief Get the Transition object
   *
   * @param current_state
   * @param event
   * @return StateMachineTransition
   */
  StateMachineTransition getTransition(const int8_t & current_state, const int8_t & event);

  /**
   * @brief This function returns the current state of the state machine
   * @return The current Platform Status of the state machine
   */
  inline as2_msgs::msg::PlatformStatus getState() {return state_;}

  /**
   * @brief Set the State of the FSM to the desired state. (THIS MAY BE USED ONLY FOR TESTING
   * PURPOSES)
   * @param state
   */
  inline void setState(as2_msgs::msg::PlatformStatus state) {state_ = state;}
  inline void setState(const int8_t & state) {state_.state = state;}

private:
  std::vector<StateMachineTransition> transitions_;
  as2_msgs::msg::PlatformStatus state_;
  as2::Node * node_ptr_;

  /**
   * @brief in this function the state machine is created based on the transitions.
   * its called in the constructor of the class.
   */
  void defineTransitions();

  rclcpp::Service<as2_msgs::srv::SetPlatformStateMachineEvent>::SharedPtr state_machine_event_srv_;

  void setStateMachineEventSrvCallback(
    const std::shared_ptr<as2_msgs::srv::SetPlatformStateMachineEvent::Request> request,
    std::shared_ptr<as2_msgs::srv::SetPlatformStateMachineEvent::Response> response);

  std::string eventToString(int8_t event);

  std::string stateToString(int8_t state);
};

}  // namespace as2

#endif  // AS2_CORE__PLATFORM_STATE_MACHINE_HPP_
