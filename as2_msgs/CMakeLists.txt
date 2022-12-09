cmake_minimum_required(VERSION 3.5)

set(PROJECT_NAME as2_msgs)
project(${PROJECT_NAME} VERSION 0.2.0)

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
  "msg/Speed.msg"
  "msg/Thrust.msg"
  "msg/TrajectoryPoint.msg"
  "msg/TrajectoryWaypoints.msg"
  "msg/TrajectoryWaypointsWithID.msg"
  "msg/MissionEvent.msg"
  "msg/NodeStatus.msg"
  "msg/TrajGenInfo.msg"
  "msg/Geofence.msg"
  "msg/Alert.msg"
  "msg/AlertEvent.msg"
  "msg/BehaviorStatus.msg"
  "msg/YawMode.msg"
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
  "srv/SendTrajectoryWaypoints.srv"
  "srv/SetControlMode.srv"
  "srv/SetOrigin.srv"
  "srv/SetPlatformStateMachineEvent.srv"
  "srv/SetSpeed.srv"
  "srv/SetGeofence.srv"
  "srv/GetGeofence.srv"
)

set(ACTION_FILES
  "action/FollowPath.action"
  "action/GoToWaypoint.action"
  "action/Land.action"
  "action/TakeOff.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSGS_FILES}
  ${SRVS_FILES} 
  ${ACTION_FILES}
  DEPENDENCIES ${PROJECT_DEPENDENCIES}
)

ament_package()
