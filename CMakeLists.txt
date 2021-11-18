cmake_minimum_required(VERSION 3.0.2)
project(mav_interface_msgs)

## Eigen 
find_package(Eigen3 REQUIRED)

## ROS msgs dependencies
find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
 FILES
 AutopilotStatus.msg
 Waypoint.msg
 Path.msg   
 FullStateStamped.msg
 FullStateTrajectory.msg
)

## Add services 
add_service_files(
  FILES
  AutopilotStatusService.srv
)

## Generate added messages
generate_messages(
 DEPENDENCIES
 geometry_msgs
 std_msgs
)

## catkin specific configuration
catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS geometry_msgs std_msgs message_runtime
)


install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)