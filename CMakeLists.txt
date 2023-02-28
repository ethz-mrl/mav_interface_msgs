# SPDX-FileCopyrightText: 2020-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-License-Identifier: CC0-1.0

cmake_minimum_required(VERSION 3.0.2)
project(mav_interface_msgs
    VERSION 0.0.0
    DESCRIPTION "Messages for interfacing with SRL MAV controllers"
    HOMEPAGE_URL "https://github.com/smartroboticslab/mav_interface_msgs"
    LANGUAGES CXX
)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    message_generation
    std_msgs
)

add_message_files(FILES
    AutopilotStatus.msg
    FullStateStamped.msg
    FullStateTrajectory.msg
    Path.msg
    Waypoint.msg
)

add_service_files(FILES
    AutopilotStatusService.srv
)

generate_messages(DEPENDENCIES
    geometry_msgs
    std_msgs
)

catkin_package(
    INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
    CATKIN_DEPENDS geometry_msgs std_msgs message_runtime
)

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)
