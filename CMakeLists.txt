cmake_minimum_required(VERSION 3.14)
project(controller_task_node)

find_package(autoware_cmake REQUIRED)
autoware_package()

set(CONTROLLER_TASK_NODE controller)
ament_auto_add_library(${CONTROLLER_TASK_NODE} SHARED
  include/controller_task_node/controller.hpp
  src/controller.cpp
)

rclcpp_components_register_node(${CONTROLLER_TASK_NODE}
  PLUGIN "controller::Controller"
  EXECUTABLE ${CONTROLLER_TASK_NODE}_exe
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
