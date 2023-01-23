/*!*******************************************************************************************
 *  \file       platform_state_machine.cpp
 *  \brief      Aerostack2 Platform State Machine implementation file.
 *  \authors    Miguel Fernandez Cortizas
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#include "platform_state_machine.hpp"

namespace as2 {
PlatformStateMachine::PlatformStateMachine(as2::Node *node) : node_ptr_(node) {
  state_.state = as2_msgs::msg::PlatformStatus::DISARMED;
  defineTransitions();

  // Initialize the srv server
  state_machine_event_srv_ = node_ptr_->create_service<as2_msgs::srv::SetPlatformStateMachineEvent>(
      node_ptr_->generate_local_name("state_machine_event"),
      std::bind(&PlatformStateMachine::setStateMachineEventSrvCallback, this, std::placeholders::_1,
                std::placeholders::_2));
}

PlatformStateMachine::~PlatformStateMachine() { state_machine_event_srv_.reset(); }

bool PlatformStateMachine::processEvent(const int8_t &event) {
  // Get the current state
  int8_t current_state = state_.state;

  // Get the transition that matches the current state and the event
  StateMachineTransition transition = getTransition(current_state, event);

  // If the transition is valid, change the state
  if (transition.transition_id == -11) {
    RCLCPP_WARN(node_ptr_->get_logger(), "Invalid transition: %s -> %s",
                stateToString(current_state).c_str(), eventToString(event).c_str());
    return false;
  }

  state_.state = transition.to_state_id;
  RCLCPP_INFO(node_ptr_->get_logger(), "Transition [%s] : New State [%s]",
              transition.transition_name.c_str(), stateToString(transition.to_state_id).c_str());

  return true;
}

bool PlatformStateMachine::processEvent(const Event &event) { return processEvent(event.event); };

StateMachineTransition PlatformStateMachine::getTransition(const int8_t &current_state,
                                                           const int8_t &event) {
  StateMachineTransition transition;
  transition.transition_id = -11;
  for (int i = 0; i < transitions_.size(); i++) {
    if (transitions_[i].from_state_id == current_state && transitions_[i].transition_id == event) {
      transition = transitions_[i];
      break;
    }
  }
  return transition;
}

void PlatformStateMachine::setStateMachineEventSrvCallback(
    const std::shared_ptr<as2_msgs::srv::SetPlatformStateMachineEvent::Request> request,
    std::shared_ptr<as2_msgs::srv::SetPlatformStateMachineEvent::Response> response) {
  response->success       = processEvent(request->event);
  response->current_state = state_;
}

void PlatformStateMachine::defineTransitions() {
  transitions_.clear();
  transitions_.reserve(11);

  // INTIAL_STATE -> [TRANSITION] -> FINAL_STATE

  // DISARMED -> [ARM] -> ARMED
  transitions_.emplace_back(StateMachineTransition{"ARM", as2_msgs::msg::PlatformStatus::DISARMED,
                                                   Event::ARM,
                                                   as2_msgs::msg::PlatformStatus::LANDED});

  // LANDED -> [DISARM] -> DISARMED
  transitions_.emplace_back(StateMachineTransition{"DISARM", as2_msgs::msg::PlatformStatus::LANDED,
                                                   Event::DISARM,
                                                   as2_msgs::msg::PlatformStatus::DISARMED});

  // LANDED -> [TAKE_OFF] -> TAKING_OFF
  transitions_.emplace_back(
      StateMachineTransition{"TAKE_OFF", as2_msgs::msg::PlatformStatus::LANDED, Event::TAKE_OFF,
                             as2_msgs::msg::PlatformStatus::TAKING_OFF});

  // TAKING_OFF -> [TOOK_OFF] -> FLYING
  transitions_.emplace_back(
      StateMachineTransition{"TOOK_OFF", as2_msgs::msg::PlatformStatus::TAKING_OFF, Event::TOOK_OFF,
                             as2_msgs::msg::PlatformStatus::FLYING});

  // FLYING -> [LAND] -> LANDING
  transitions_.emplace_back(StateMachineTransition{"LAND", as2_msgs::msg::PlatformStatus::FLYING,
                                                   Event::LAND,
                                                   as2_msgs::msg::PlatformStatus::LANDING});

  // LANDING -> [LANDED] -> LANDED
  transitions_.emplace_back(StateMachineTransition{"LANDED", as2_msgs::msg::PlatformStatus::LANDING,
                                                   Event::LANDED,
                                                   as2_msgs::msg::PlatformStatus::LANDED});

  // EMERGENCY TRANSITIONS
  transitions_.emplace_back(
      StateMachineTransition{"EMERGENCY", as2_msgs::msg::PlatformStatus::DISARMED, Event::EMERGENCY,
                             as2_msgs::msg::PlatformStatus::EMERGENCY});
  transitions_.emplace_back(
      StateMachineTransition{"EMERGENCY", as2_msgs::msg::PlatformStatus::LANDED, Event::EMERGENCY,
                             as2_msgs::msg::PlatformStatus::EMERGENCY});
  transitions_.emplace_back(
      StateMachineTransition{"EMERGENCY", as2_msgs::msg::PlatformStatus::TAKING_OFF,
                             Event::EMERGENCY, as2_msgs::msg::PlatformStatus::EMERGENCY});
  transitions_.emplace_back(
      StateMachineTransition{"EMERGENCY", as2_msgs::msg::PlatformStatus::FLYING, Event::EMERGENCY,
                             as2_msgs::msg::PlatformStatus::EMERGENCY});
  transitions_.emplace_back(
      StateMachineTransition{"EMERGENCY", as2_msgs::msg::PlatformStatus::LANDING, Event::EMERGENCY,
                             as2_msgs::msg::PlatformStatus::EMERGENCY});
}

std::string PlatformStateMachine::eventToString(int8_t event) {
  switch (event) {
    case as2::Event::EMERGENCY:
      return "EMERGENCY";
      break;
    case as2::Event::ARM:
      return "ARM";
      break;
    case as2::Event::DISARM:
      return "DISARM";
      break;
    case as2::Event::TAKE_OFF:
      return "TAKE_OFF";
      break;
    case as2::Event::TOOK_OFF:
      return "TOOK_OFF";
      break;
    case as2::Event::LAND:
      return "LAND";
      break;
    case as2::Event::LANDED:
      return "LANDED";
      break;
    default:
      return "UNKNOWN";
      break;
  }
}

std::string PlatformStateMachine::stateToString(int8_t state) {
  switch (state) {
    case as2_msgs::msg::PlatformStatus::EMERGENCY:
      return "EMERGENCY";
      break;
    case as2_msgs::msg::PlatformStatus::DISARMED:
      return "DISARMED";
      break;
    case as2_msgs::msg::PlatformStatus::LANDED:
      return "LANDED";
      break;
    case as2_msgs::msg::PlatformStatus::TAKING_OFF:
      return "TAKING_OFF";
      break;
    case as2_msgs::msg::PlatformStatus::FLYING:
      return "FLYING";
      break;
    case as2_msgs::msg::PlatformStatus::LANDING:
      return "LANDING";
      break;
    default:
      return "UNKNOWN";
      break;
  }
}

};  // namespace as2
