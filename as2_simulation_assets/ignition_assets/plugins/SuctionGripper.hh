/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef MBZIRC_IGN_SUCTIONGRIPPER_HH_
#define MBZIRC_IGN_SUCTIONGRIPPER_HH_

#include <memory>
#include <ignition/gazebo/System.hh>

namespace mbzirc
{
  class SuctionGripperPrivate;

  ////////////////////////////////////////////////////
  /// \brief This plugin implements a suction gripper.
  /// When the gripper makes contact with an item it picks up the item.
  /// One can command the release of the item through a configurable topic.
  /// The plugin requires a contact sensor to detect when an item is in contact.
  ///
  /// # Parameters
  /// <parent_link>
  /// The link to which this gripper should be attached. [string]
  /// <contact_sensor_topic>
  /// The topic to listen to for contact sensor inputs. [string]
  /// <command_topic>
  /// The topic to listen to for commands. [string]
  ///
  /// # Subscribers
  /// <contact_sensor_topic> used to detect when an item is in
  /// contact. [ignition::msgs::Contacts]
  /// <command_topic> - used to command the release of an item. Sending a False
  /// to this topic will cause the suction to stop. If there is an object then
  /// it will be released, otherwise new objects will not be picked up. Sending
  /// a true signal again will cause the suction to start again. By default
  /// suction is on. [igntion::msgs::Boolean]
  class SuctionGripperPlugin:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate
  {
    public: SuctionGripperPlugin();

    public: ~SuctionGripperPlugin();

    public: void Configure(const ignition::gazebo::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        ignition::gazebo::EntityComponentManager &_ecm,
        ignition::gazebo::EventManager &_eventMgr) override;

    public: void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
        ignition::gazebo::EntityComponentManager &_ecm) override;

    public: std::unique_ptr<SuctionGripperPrivate> dataPtr;
  };
}

#endif