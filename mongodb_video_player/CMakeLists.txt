cmake_minimum_required(VERSION 2.8.3)
project(mongodb_video_player)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  cv_bridge
  rospy
  mongodb_media_server
  sensor_msgs
  std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  VideoPlayerService.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

#############
## Install ##
#############

install(PROGRAMS
  scripts/video_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
