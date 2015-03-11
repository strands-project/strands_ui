cmake_minimum_required(VERSION 2.8.3)
project(music_player)

find_package(catkin REQUIRED COMPONENTS
    genmsg
    actionlib
    actionlib_msgs
    rospy
    pygame_managed_player
    mongodb_media_server
)

add_service_files(
    FILES
    MusicPlayerService.srv
)

generate_messages(
    DEPENDENCIES
)

catkin_package()

#############
## Install ##
#############

install(PROGRAMS
  scripts/music_player_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
