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


#include <gz/plugin/Register.hh>


#include <gz/transport/Node.hh>

#include <gz/msgs/contacts.pb.hh>

#include <gz/msgs.hh>

#include <gz/gazebo/components.hh>
#include <gz/gazebo/Model.hh>

#include "SuctionGripper.hpp"

// using mbzirc::SuctionGripperPrivate;
// using mbzirc::SuctionGripperPlugin;
// using ignition::gazebo;
// using gazebo::System;

// using ::mbzric::SuctionGripperPrivate;
// using ::mbzric::SuctionGripperPlugin;
// using ::ignition::gazebo::System;

// using namespace mbzirc;
// using namespace ignition;
// using namespace gazebo;

class mbzirc::SuctionGripperPrivate
{
/// \brief The item being moved

public:
  Entity childItem {kNullEntity};

/// \brief The gripper link name

public:
  std::string linkName;

/// \brief Used to store the joint when we attach to an object

public:
  Entity joint {kNullEntity};

/// \brief The gripper link entity

public:
  Entity gripperEntity {kNullEntity};

/// \brief The transport node

public:
  transport::Node node;

/// \brief Used for determining when the suction is on.

public:
  bool suctionOn {false};

/// \brief Set to true when we detect the suction gripper is in contact

public:
  bool pendingJointCreation {false};

/// \brief True when we are holding an object

public:
  bool jointCreated {false};

/// \brief mutex for accessing member variables

public:
  std::mutex mtx;

/// \brief Two-dimensional array of contact points

public:
  std::array<std::array<Entity, 3>, 3> contacts;

/// \brief Publisher for contact points

public:
  transport::Node::Publisher contactPublisherCenter;

public:
  transport::Node::Publisher contactPublisherLeft;

public:
  transport::Node::Publisher contactPublisherRight;

public:
  transport::Node::Publisher contactPublisherTop;

public:
  transport::Node::Publisher contactPublisherBottom;

/// \brief Callback for when contact is made

public:
  void OnContact(
    int idx0, int idx1,
    const gz::msgs::Contacts & _msg)
  {
    std::lock_guard<std::mutex> lock(this->mtx);

    if (_msg.contact_size()) {
      auto contact = _msg.contact(0);
      this->contacts[idx0][idx1] = contact.collision2().id();
    } else {
      this->contacts[idx0][idx1] = kNullEntity;
    }
  }

/// \brief Command callback

public:
  void OnCmd(const gz::msgs::Boolean & _suctionOn)
  {
    std::lock_guard<std::mutex> lock(this->mtx);
    this->suctionOn = _suctionOn.data();
  }
};


//////////////////////////////////////////////////
SuctionGripperPlugin::SuctionGripperPlugin()
: dataPtr(new SuctionGripperPrivate)
{
  for (size_t ii = 0; ii < 3; ++ii) {
    for (size_t jj = 0; jj < 3; ++jj) {
      this->dataPtr->contacts[ii][jj] = kNullEntity;
    }
  }
}

//////////////////////////////////////////////////
SuctionGripperPlugin::~SuctionGripperPlugin()
{
}

//////////////////////////////////////////////////
void SuctionGripperPlugin::Configure(
  const Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  EntityComponentManager & _ecm,
  EventManager & /*_eventMgr*/)
{
  if (_sdf->HasElement("parent_link")) {
    this->dataPtr->linkName = _sdf->Get<std::string>("parent_link");
  } else {
    ignerr << "Please specify a link name" << std::endl;
    return;
  }

  Model model(_entity);
  this->dataPtr->gripperEntity = model.LinkByName(_ecm, this->dataPtr->linkName);
  if (this->dataPtr->gripperEntity == kNullEntity) {
    ignerr << "Could not find link named "
           << this->dataPtr->linkName << std::endl;
    return;
  }

  if (_sdf->HasElement("contact_sensor_topic_prefix")) {
    auto prefix = _sdf->Get<std::string>("contact_sensor_topic_prefix");

    std::function<void(const gz::msgs::Contacts &)> callback_01 =
      std::bind(
      &SuctionGripperPrivate::OnContact, this->dataPtr.get(), 0, 1,
      std::placeholders::_1);
    this->dataPtr->node.Subscribe(prefix + "/contact_sensor_01", callback_01);

    std::function<void(const gz::msgs::Contacts &)> callback_11 =
      std::bind(
      &SuctionGripperPrivate::OnContact, this->dataPtr.get(), 1, 1,
      std::placeholders::_1);
    this->dataPtr->node.Subscribe(prefix + "/contact_sensor_11", callback_11);

    std::function<void(const gz::msgs::Contacts &)> callback_21 =
      std::bind(
      &SuctionGripperPrivate::OnContact, this->dataPtr.get(), 2, 1,
      std::placeholders::_1);
    this->dataPtr->node.Subscribe(prefix + "/contact_sensor_21", callback_21);

    std::function<void(const gz::msgs::Contacts &)> callback_10 =
      std::bind(
      &SuctionGripperPrivate::OnContact, this->dataPtr.get(), 1, 0,
      std::placeholders::_1);
    this->dataPtr->node.Subscribe(prefix + "/contact_sensor_10", callback_10);

    std::function<void(const gz::msgs::Contacts &)> callback_12 =
      std::bind(
      &SuctionGripperPrivate::OnContact, this->dataPtr.get(), 1, 2,
      std::placeholders::_1);
    this->dataPtr->node.Subscribe(prefix + "/contact_sensor_12", callback_12);


    this->dataPtr->contactPublisherCenter =
      this->dataPtr->node.Advertise<msgs::Boolean>(prefix + "/contacts/center");
    this->dataPtr->contactPublisherLeft =
      this->dataPtr->node.Advertise<msgs::Boolean>(prefix + "/contacts/left");
    this->dataPtr->contactPublisherRight =
      this->dataPtr->node.Advertise<msgs::Boolean>(prefix + "/contacts/right");
    this->dataPtr->contactPublisherTop =
      this->dataPtr->node.Advertise<msgs::Boolean>(prefix + "/contacts/top");
    this->dataPtr->contactPublisherBottom =
      this->dataPtr->node.Advertise<msgs::Boolean>(prefix + "/contacts/bottom");
  } else {
    ignerr << "Please specify a contact_sensor_topic_prefix" << std::endl;
    return;
  }

  if (_sdf->HasElement("command_topic")) {
    auto topic = _sdf->Get<std::string>("command_topic");
    this->dataPtr->node.Subscribe(
      topic,
      &SuctionGripperPrivate::OnCmd,
      this->dataPtr.get());
  } else {
    ignerr << "Please specify a command_topic" << std::endl;
    return;
  }
}


//////////////////////////////////////////////////
void SuctionGripperPlugin::PreUpdate(
  const UpdateInfo & _info,
  EntityComponentManager & _ecm)
{
  if (_info.paused) {return;}
  std::lock_guard<std::mutex> lock(this->dataPtr->mtx);

  msgs::Boolean contact;

  // If the gripper is engaged and holding an object, return contacts as true
  if (this->dataPtr->jointCreated) {
    contact.set_data(true);
    this->dataPtr->contactPublisherCenter.Publish(contact);
    this->dataPtr->contactPublisherLeft.Publish(contact);
    this->dataPtr->contactPublisherRight.Publish(contact);
    this->dataPtr->contactPublisherTop.Publish(contact);
    this->dataPtr->contactPublisherBottom.Publish(contact);
  } else {
    contact.set_data(this->dataPtr->contacts[1][1] != kNullEntity);
    this->dataPtr->contactPublisherCenter.Publish(contact);

    contact.set_data(this->dataPtr->contacts[1][0] != kNullEntity);
    this->dataPtr->contactPublisherLeft.Publish(contact);

    contact.set_data(this->dataPtr->contacts[1][2] != kNullEntity);
    this->dataPtr->contactPublisherRight.Publish(contact);

    contact.set_data(this->dataPtr->contacts[0][1] != kNullEntity);
    this->dataPtr->contactPublisherTop.Publish(contact);

    contact.set_data(this->dataPtr->contacts[2][1] != kNullEntity);
    this->dataPtr->contactPublisherBottom.Publish(contact);
  }


  if (!this->dataPtr->jointCreated && this->dataPtr->suctionOn) {
    // check that two sensors are making contact with the same object
    auto checkContacts =
      [&](std::pair<int, int> idx0, std::pair<int, int> idx1)->bool {
        auto contact0 = this->dataPtr->contacts[idx0.first][idx0.second];
        auto contact1 = this->dataPtr->contacts[idx1.first][idx1.second];
        return
          contact0 != kNullEntity &&
          contact1 != kNullEntity &&
          contact0 == contact1;
      };


    bool contactMade =
      (checkContacts({1, 1}, {1, 0}) ||                   // Center + left
      checkContacts({1, 1}, {1, 2}) ||                    // Center + right
      checkContacts({1, 1}, {0, 1}) ||                    // Center + top
      checkContacts({1, 1}, {2, 1}));                    // Center + bottom

    if (contactMade) {
      this->dataPtr->pendingJointCreation = true;
      this->dataPtr->childItem = this->dataPtr->contacts[1][1];
    } else if (checkContacts({1, 0}, {1, 2})) {         // left + right
      this->dataPtr->pendingJointCreation = true;
      this->dataPtr->childItem = this->dataPtr->contacts[1][0];
    } else if (checkContacts({0, 1}, {2, 1})) {         // top + bottom
      this->dataPtr->pendingJointCreation = true;
      this->dataPtr->childItem = this->dataPtr->contacts[0][1];
    }
  }

  // Clear contacts
  for (size_t ii = 0; ii < 3; ++ii) {
    for (size_t jj = 0; jj < 3; ++jj) {
      this->dataPtr->contacts[ii][jj] = kNullEntity;
    }
  }

  if (this->dataPtr->pendingJointCreation) {
    // If we need to create a new joint
    this->dataPtr->pendingJointCreation = false;
    this->dataPtr->joint = _ecm.CreateEntity();
    auto parentLink = _ecm.ParentEntity(this->dataPtr->childItem);
    _ecm.CreateComponent(
      this->dataPtr->joint,
      components::DetachableJoint(
        {this->dataPtr->gripperEntity,
          parentLink, "fixed"}));
    igndbg << "Created joint between gripper and "
           << this->dataPtr->childItem
           << std::endl << "at time step " << _info.simTime.count() << std::endl;
    this->dataPtr->jointCreated = true;
  }

  if (!this->dataPtr->suctionOn && this->dataPtr->jointCreated) {
    // If we have an item and were commanded to release it
    _ecm.RequestRemoveEntity(this->dataPtr->joint);
    this->dataPtr->joint = kNullEntity;
    this->dataPtr->jointCreated = false;
    igndbg << "Remove joint between gripper and "
           << this->dataPtr->childItem
           << std::endl << "at time step " << _info.simTime.count() << std::endl;
  }
}


IGNITION_ADD_PLUGIN(
  mbzirc::SuctionGripperPlugin,
  ignition::gazebo::System,
  mbzirc::SuctionGripperPlugin::ISystemConfigure,
  mbzirc::SuctionGripperPlugin::ISystemPreUpdate)
