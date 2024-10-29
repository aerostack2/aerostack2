/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "Common.hpp"

#include <string>

#include <gz/common/Console.hh>
#include <gz/math/Rand.hh>
#include <gz/math/eigen3/Conversions.hh>

#include <gz/sim/components/ChildLinkName.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/AngularVelocity.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE
{
namespace systems
{
namespace multicopter_control
{

//////////////////////////////////////////////////
RotorConfiguration loadRotorConfiguration(
  const EntityComponentManager & _ecm,
  const sdf::ElementPtr & _sdf,
  const Model & _model,
  const Entity & _comLink)
{
  RotorConfiguration out;
  std::size_t count = 0;
  for (sdf::ElementPtr elem = _sdf->GetFirstElement(); elem;
    elem = elem->GetNextElement("rotor"), ++count)
  {
    Rotor rotor;
    if (!elem->HasElement("jointName")) {
      gzerr << "Please specify jointName for rotor index " << count
            << std::endl;
      continue;
    }
    const std::string jointName = elem->Get<std::string>("jointName");
    const Entity joint = _ecm.EntityByComponents(
      components::Joint(), components::Name(jointName),
      components::ParentEntity(_model.Entity()));
    if (kNullEntity == joint) {
      gzerr << "Joint with name " << jointName << " could not be found in "
            << "model " << _model.Name(_ecm) << " while processing rotor "
            << "index " << count << std::endl;
      continue;
    }

    // Calculate the armLength and angle of propeller. The angle of a
    // propeller is the angle about the z axis from the x axis of the frame
    // attached to _comLink to the joint.

    // First, get the pose of the joint w.r.t _comLink
    const std::string childLinkName =
      _ecm.Component<components::ChildLinkName>(joint)->Data();

    const Entity childLink = _ecm.EntityByComponents(
      components::Link(), components::Name(childLinkName),
      components::ParentEntity(_model.Entity()));

    if (kNullEntity == childLink) {
      gzerr << "Child link of joint " << jointName << " with name "
            << childLinkName << " could not be found in  model "
            << _model.Name(_ecm) << " while processing rotor index " << count
            << std::endl;
      continue;
    }

    const math::Pose3d jointPose =
      _ecm.Component<components::Pose>(joint)->Data();
    const math::Pose3d childLinkPose =
      _ecm.Component<components::Pose>(childLink)->Data();
    const math::Pose3d comLinkPose =
      _ecm.Component<components::Pose>(_comLink)->Data();

    const math::Pose3d jointPoseComLink =
      comLinkPose.Inverse() * (childLinkPose * jointPose);

    // Project jointPoseComLink into the rotor plane, which is assumed to be
    // the XY plane.
    math::Vector3d unitZ = math::Vector3d::UnitZ;
    math::Vector3d rotorProj =
      jointPoseComLink.Pos() - (unitZ.Dot(jointPoseComLink.Pos()) * unitZ);

    rotor.armLength = rotorProj.Length();
    math::Quaterniond rot;
    rot.SetFrom2Axes(math::Vector3d::UnitX, rotorProj);
    rotor.angle = rot.Yaw();

    std::cout << "The armLength of rotor " << count << " is: " << rotor.armLength << std::endl;
    std::cout << "The angle of rotor " << count << " is: " << rotor.angle << std::endl;

    if (!elem->HasElement("forceConstant")) {
      gzerr << "Please specify forceConstant for rotor index " << count
            << std::endl;
      continue;
    }
    rotor.forceConstant = elem->Get<double>("forceConstant");

    // forceConstant assumes that the rotor's thrust is along the body's +z
    // direction. However, UAVs have their rotors oriented such they are not
    // orthogonal to the rotor plane (the COM link frame's xy plane). To
    // account for the difference in thrust, we multiply the forceConstant
    // with the projection of the rotor's vector onto the +z axis of the
    // COM link frame.
    auto axis = _ecm.Component<components::JointAxis>(joint);
    if (nullptr != axis) {
      math::Vector3d xyzInJoint;
      axis->Data().ResolveXyz(xyzInJoint);

      auto xyzInComLink = comLinkPose.Rot().Inverse() *
        (childLinkPose.Rot() * jointPose.Rot() * xyzInJoint);

      // The projection onto the +z axis is just the z component of the
      // xyzInComLink vector
      rotor.forceConstant *= xyzInComLink.Z();
    }


    if (!elem->HasElement("momentConstant")) {
      gzerr << "Please specify momentConstant for rotor index " << count
            << std::endl;
      continue;
    }
    rotor.momentConstant = elem->Get<double>("momentConstant");

    if (!elem->HasElement("direction")) {
      gzerr << "Please specify direction for rotor index " << count
            << std::endl;
      continue;
    }
    rotor.direction = elem->Get<int>("direction");

    out.push_back(rotor);
  }

  return out;
}

//////////////////////////////////////////////////
void createFrameDataComponents(
  EntityComponentManager & _ecm,
  const Entity & _link)
{
  // Create an angular velocity component if one is not present.
  if (!_ecm.Component<components::AngularVelocity>(_link)) {
    _ecm.CreateComponent(_link, components::AngularVelocity());
  }
}

//////////////////////////////////////////////////
std::optional<FrameData> getFrameData(
  const EntityComponentManager & _ecm,
  const Entity & _link,
  const NoiseParameters & _noise)
{
  auto angVelComp = _ecm.Component<components::AngularVelocity>(_link);

  if (!angVelComp) {
    gzerr << "AngularVelocity component not found on link entity " << _link
          << std::endl;
    return std::nullopt;
  }

  auto applyNoise = [](Eigen::Vector3d & _val, const Eigen::Vector3d & _mean,
      const Eigen::Vector3d & _stdDev)
    {
      for (int i = 0; i < 3; ++i) {
        if (_stdDev(i) > 0) {
          _val(i) += math::Rand::DblNormal(_mean(i), _stdDev(i));
        }
      }
    };

  auto frameData = std::make_optional<FrameData>(
    {math::eigen3::convert(angVelComp->Data())});

  // applyNoise(
  //   frameData->angularVelocityBody, _noise.angularVelocityMean,
  //   _noise.angularVelocityStdDev);

  return frameData;
}

}
}
}
}
}
