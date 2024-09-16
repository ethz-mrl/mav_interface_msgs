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
    tf
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
    tf
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
    LIBRARIES
        ${PROJECT_NAME}
)

add_library(${PROJECT_NAME} STATIC src/conversions.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_11)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Eigen3::Eigen) # TODO: why is Eigen needed here?

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME} DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
