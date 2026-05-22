// Copyright 2026 Universidad Politécnica de Madrid
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
 *  \file       state_interface.hpp
 *  \brief      state interface header file
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#ifndef AS2_CORE__STATE_INTERFACE_HPP_
#define AS2_CORE__STATE_INTERFACE_HPP_

#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <utility>

#include "as2_core/node.hpp"
#include "as2_core/names/topics.hpp"
#include "rclcpp/subscription.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

class StateInterface
{
public:
  struct ActiveRegisterEntryBase
  {
    std::string key;
    virtual void clear() = 0;
    virtual ~ActiveRegisterEntryBase() = default;
  };
  template<typename T>
  struct ActiveRegisterEntry : public ActiveRegisterEntryBase
  {
    std::shared_ptr<T> value_ptr;
    typename rclcpp::Subscription<T>::SharedPtr subscription_ptr;
    void clear()
    {
      subscription_ptr.reset();
      value_ptr.reset();
    }
  };

  struct AvailableRegisterEntryBase
  {
    virtual ~AvailableRegisterEntryBase() = default;
    virtual std::shared_ptr<ActiveRegisterEntryBase> create(
      rclcpp::Node * node,
      const std::string & key) = 0;
  };

  template<typename T>
  struct RegisterEntry : public AvailableRegisterEntryBase
  {
    std::shared_ptr<ActiveRegisterEntryBase> create(
      rclcpp::Node * node,
      const std::string & key) override
    {
      auto entry = std::make_shared<ActiveRegisterEntry<T>>();
      entry->key = key;
      entry->value_ptr = std::make_shared<T>();
      entry->subscription_ptr = node->create_subscription<T>(
        key, rclcpp::SensorDataQoS(),
        [value_ptr = entry->value_ptr](const typename T::SharedPtr msg) {
          *value_ptr = *msg;
        });
      return entry;
    }
  };

  struct AddAvailableEntry
  {
    AddAvailableEntry(
      const std::string & name,
      std::function<std::shared_ptr<AvailableRegisterEntryBase>()> factory)
    {
      get_registry()[name] = factory;
    }
  };

  static std::unordered_map<std::string,
    std::function<std::shared_ptr<AvailableRegisterEntryBase>()>> &
  get_registry()
  {
    static std::unordered_map<std::string,
      std::function<std::shared_ptr<AvailableRegisterEntryBase>()>>
    registry;
    return registry;
  }

private:
  rclcpp::Node * node_ptr_;
  std::vector<std::shared_ptr<ActiveRegisterEntryBase>> active_entries_;

  bool add_to_registry(const std::string & key)
  {
    auto factory_it = get_registry().find(key);
    if (factory_it == get_registry().end()) {
      RCLCPP_ERROR(
        node_ptr_->get_logger(), "Name [%s] is not available for registration",
        key.c_str());
      return false;
    }
    auto factory = factory_it->second;
    auto entry = factory();
    auto active_entry = entry->create(node_ptr_, key);

    bool success = active_entry != nullptr;
    if (success) {
      active_entries_.push_back(std::move(active_entry));
      RCLCPP_INFO(node_ptr_->get_logger(), "Successfully registered name [%s]", key.c_str());
    } else {
      RCLCPP_ERROR(
        node_ptr_->get_logger(), "Failed to create subscription for name [%s]",
        key.c_str());
    }

    return success;
  }

public:
  ~StateInterface() = default;

  template<typename T>
  T get_value(const std::string & key) const
  {
    for (auto & base_entry : active_entries_) {
      if (base_entry->key == key) {
        auto typed_entry = std::dynamic_pointer_cast<ActiveRegisterEntry<T>>(base_entry);
        if (!typed_entry) {
          RCLCPP_ERROR(
            node_ptr_->get_logger(), "[State Interface] Type mismatch for key [%s]", key.c_str());
          throw std::runtime_error("Type mismatch");
        }
        return *(typed_entry->value_ptr);
      }
    }
    RCLCPP_ERROR(
      node_ptr_->get_logger(), "[State Interface] Key [%s] is not registered",
      key.c_str());
    throw std::runtime_error("Key not registered");
  }

  bool configure(rclcpp::Node * node_ptr, const std::vector<std::string> & keys)
  {
    node_ptr_ = node_ptr;
    for (const auto & key : keys) {
      if (!add_to_registry(key)) {
        RCLCPP_ERROR(
          node_ptr_->get_logger(), "[State Interface] Failed to register key [%s]", key.c_str());
        return false;
      }
    }
    return true;
  }

  void clear()
  {
    active_entries_.clear();
  }
};

#define _SI_CONCAT_IMPL(a, b) a ## b
#define _SI_CONCAT(a, b) _SI_CONCAT_IMPL(a, b)

#define REGISTER_STATE_ENTRY(NAME, TYPE) \
  namespace { \
  StateInterface::AddAvailableEntry _SI_CONCAT(_si_reg_, __LINE__) { \
    NAME, []() -> std::shared_ptr<StateInterface::AvailableRegisterEntryBase> { \
      return std::make_shared<StateInterface::RegisterEntry<TYPE>>(); \
    } \
  }; \
  }

REGISTER_STATE_ENTRY(
  as2_names::topics::self_localization::pose,
  geometry_msgs::msg::PoseStamped)

REGISTER_STATE_ENTRY(
  as2_names::topics::self_localization::twist,
  geometry_msgs::msg::TwistStamped)

REGISTER_STATE_ENTRY(
  as2_names::topics::sensor_measurements::battery,
  sensor_msgs::msg::BatteryState)

#endif  // AS2_CORE__STATE_INTERFACE_HPP_
