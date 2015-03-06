cmake_minimum_required(VERSION 2.8.3)
project(mongodb_media_server)

find_package(catkin REQUIRED COMPONENTS
  mongodb_store
  rospy
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mongodb_media_server
#  CATKIN_DEPENDS mongodb_store rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############
install(DIRECTORY scripts/
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        USE_SOURCE_PERMISSIONS)
        
        
install(DIRECTORY www
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS)
        


#############
## Testing ##
#############

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
