# SPDX-FileCopyrightText: 2020-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-License-Identifier: CC0-1.0

cmake_minimum_required(VERSION 3.0..3.16 FATAL_ERROR)
project(mav_interface_msgs
    VERSION 1.0.0
    DESCRIPTION "Messages for interfacing with SRL MAV controllers"
    HOMEPAGE_URL "https://github.com/smartroboticslab/mav_interface_msgs"
    LANGUAGES CXX
)

# Find package dependencies.
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    message_generation
    roscpp
    geometry_msgs
    nav_msgs
    std_msgs
)

# Generate messages. Must be before the call to catkin_package().
add_message_files(FILES
    AutopilotStatus.msg
    FullStateStamped.msg
    FullStateTrajectory.msg
    OdomAndTrajectory.msg
    Path.msg
    Waypoint.msg
)

add_service_files(FILES
    AutopilotStatusService.srv
)

generate_messages(DEPENDENCIES
    geometry_msgs
    nav_msgs
    std_msgs
)

# Create the package.
catkin_package(
    CATKIN_DEPENDS
        geometry_msgs
        nav_msgs
        std_msgs
        message_runtime
    DEPENDS # Names must be the prefixes of the *_INCLUDE_DIRS and *_LIBRARIES variables.
        EIGEN3
    INCLUDE_DIRS
        include
)

install(DIRECTORY include/${PROJECT_NAME} DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
