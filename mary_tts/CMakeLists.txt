cmake_minimum_required(VERSION 2.8.3)
project(mary_tts)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
    actionlib
    actionlib_msgs
    message_generation 
    roslib
    rospy 
    std_msgs 
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
#catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################


## Generate services in the 'srv' folder
add_service_files(
  FILES
  ros_mary.srv
  SetVoice.srv
  SetLocale.srv
)

add_action_files(
  DIRECTORY action
  FILES marytts.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  
  actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros-mary-tts
	CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    roslib
    rospy 
    std_msgs 
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## Declare a cpp library
# add_library(ros-mary-tts
#   src/${PROJECT_NAME}/ros-mary-tts.cpp
# )

## Declare a cpp executable
# add_executable(ros-mary-tts_node src/ros-mary-tts_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(ros-mary-tts_node ros-mary-tts_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(ros-mary-tts_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/marybridge.py
  scripts/speak_webserver.py
  scripts/marytts-server.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY marytts-5.0/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/marytts-5.0
  USE_SOURCE_PERMISSIONS
)


install(DIRECTORY static/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/static
) 


install(DIRECTORY templates/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/templates
) 



#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ros-mary-tts.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
