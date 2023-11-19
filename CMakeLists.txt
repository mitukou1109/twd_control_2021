cmake_minimum_required(VERSION 3.0.2)
project(twd_control_2021)
set(CMAKE_CXX_FLAGS "-std=c++17")

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf2
  tf2_ros
  agv_control_msgs
)

add_message_files(FILES
  MotorSpeedControllerFeedback.msg
  MotorSpeedControllerParam.msg
)
generate_messages(DEPENDENCIES
  geometry_msgs
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/MotorSpeedController.cfg
)

catkin_package(
  CATKIN_DEPENDS
    dynamic_reconfigure
    geometry_msgs
    message_runtime
    nav_msgs
    roscpp
    rospy
    std_msgs
    tf2
    tf2_ros
    urg_node
    agv_control_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(manual_controller
  src/manual_controller.cpp
)
add_dependencies(manual_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(manual_controller
  ${catkin_LIBRARIES}
)

add_executable(twd_robot_state_publisher
  src/twd_robot_state_publisher.cpp
)
target_link_libraries(twd_robot_state_publisher
  ${catkin_LIBRARIES}
)