# SPDX-FileCopyrightText: 2020-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-License-Identifier: CC0-1.0

cmake_minimum_required(VERSION 3.8)
project(mav_interface_msgs
    VERSION 2.0.0
    DESCRIPTION "Messages for interfacing with SRL MAV controllers"
    HOMEPAGE_URL "https://github.com/smartroboticslab/mav_interface_msgs"
    LANGUAGES CXX
)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/AutopilotStatus.msg"
    "msg/FullStateStamped.msg"
    "msg/FullStateTrajectory.msg"
    "msg/OdomAndTrajectory.msg"
    "msg/Path.msg"
    "msg/Waypoint.msg"
    "srv/AutopilotStatusService.srv"
    DEPENDENCIES
    geometry_msgs
    nav_msgs
    std_msgs
    tf
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
