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

include_directories(${Eigen3_INCLUDE_DIRS})

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME}
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_11)
target_compile_options(${PROJECT_NAME} INTERFACE -Wall -Wextra -Wpedantic)

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

# # TARGET DEPENDENCIES
AMENT_TARGET_DEPENDENCIES(
    ament_cmake
    eigen3_cmake_module
    Eigen3
    geometry_msgs
    nav_msgs
    std_msgs
    tf
)

# EXPORT DEPENDENCIES
AMENT_EXPORT_DEPENDENCIES(
    ament_cmake
    rosidl_default_runtime
    eigen3_cmake_module
    Eigen3
    nav_msgs
    geometry_msgs
    std_msgs
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
