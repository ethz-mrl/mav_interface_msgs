# SPDX-FileCopyrightText: 2020-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-License-Identifier: CC0-1.0

cmake_minimum_required(VERSION 3.8)
project(mav_interface_msgs
    VERSION 2.0.0
    DESCRIPTION "Messages for interfacing with SRL MAV controllers"
    HOMEPAGE_URL "https://github.com/smartroboticslab/mav_interface_msgs"
    LANGUAGES C CXX
)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
    "msg/AutopilotStatus.msg"
    "msg/FullStateStamped.msg"
    "msg/FullStateTrajectory.msg"
    "msg/OdomAndTrajectory.msg"
    "msg/Path.msg"
    "msg/Waypoint.msg"
    "srv/AutopilotStatusService.srv"
    )

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    DEPENDENCIES
    geometry_msgs
    nav_msgs
    std_msgs
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


add_library(${PROJECT_NAME}_cpp STATIC src/conversions.cpp)
target_compile_features(${PROJECT_NAME}_cpp PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME}_cpp PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME}_cpp PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_cpp Eigen3::Eigen)

# Link against the generated interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(${PROJECT_NAME}_cpp "${cpp_typesupport_target}")

ament_target_dependencies(${PROJECT_NAME}_cpp 
  geometry_msgs
  nav_msgs
  std_msgs
)

install(TARGETS ${PROJECT_NAME}_cpp
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(geometry_msgs nav_msgs std_msgs)

ament_package()
