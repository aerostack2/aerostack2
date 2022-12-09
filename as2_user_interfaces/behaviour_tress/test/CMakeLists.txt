### TEST ###
find_package(geometry_msgs REQUIRED)

add_executable(test_port port.cpp)
target_link_libraries(test_port BT::behaviortree_cpp_v3)
ament_target_dependencies(test_port geometry_msgs)


add_executable(test_port_pose port_pose.cpp)
target_link_libraries(test_port_pose BT::behaviortree_cpp_v3)
ament_target_dependencies(test_port_pose geometry_msgs)
############
