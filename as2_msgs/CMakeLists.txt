cmake_minimum_required(VERSION 3.5)

set(PROJECT_NAME as2_msgs)
project(${PROJECT_NAME} VERSION 1.0.0)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(PROJECT_DEPENDENCIES
  std_msgs
  geometry_msgs
  geographic_msgs
  builtin_interfaces
  nav_msgs
  action_msgs
  sensor_msgs
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

set(MSGS_FILES
  "msg/ControllerInfo.msg"
  "msg/ControlMode.msg"
  "msg/FollowTargetInfo.msg"
  "msg/PlatformInfo.msg"
  "msg/PlatformStateMachineEvent.msg"
  "msg/PlatformStatus.msg"
  "msg/PoseStampedWithID.msg"
  "msg/PoseWithID.msg"
  "msg/Speed.msg"
  "msg/Thrust.msg"
  "msg/TrajectoryPoint.msg"
  "msg/MissionEvent.msg"
  "msg/MissionUpdate.msg"
  "msg/NodeStatus.msg"
  "msg/TrajGenInfo.msg"
  "msg/AlertEvent.msg"
  "msg/BehaviorStatus.msg"
  "msg/YawMode.msg"
  "msg/GimbalControl.msg"
  "msg/Geozone.msg"
  "msg/PolygonList.msg"
)

set(SRVS_FILES
  "srv/DynamicFollower.srv"
  "srv/DynamicLand.srv"
  "srv/GeopathToPath.srv"
  "srv/GetOrigin.srv"
  "srv/ListControlModes.srv"
  "srv/PackagePickUp.srv"
  "srv/PackageUnPick.srv"
  "srv/PathToGeopath.srv"
  "srv/SetControlMode.srv"
  "srv/SetOrigin.srv"
  "srv/SetPlatformStateMachineEvent.srv"
  "srv/SetSpeed.srv"
  "srv/AddStaticTransform.srv"
  "srv/AddStaticTransformGps.srv"
  "srv/GetGeozone.srv"
  "srv/SetGeozone.srv"
)

set(ACTION_FILES
  "action/DetectArucoMarkers.action"
  "action/FollowPath.action"
  "action/GoToWaypoint.action"
  "action/Land.action"
  "action/Takeoff.action"
  "action/GeneratePolynomialTrajectory.action"
  "action/SetArmingState.action"
  "action/SetOffboardMode.action"
  "action/FollowReference.action"
  "action/PointGimbal.action"
  "action/NavigateToPoint.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSGS_FILES}
  ${SRVS_FILES}
  ${ACTION_FILES}
  DEPENDENCIES ${PROJECT_DEPENDENCIES}
)

if(BUILD_TESTING)
  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake()
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()
endif()

ament_package()
