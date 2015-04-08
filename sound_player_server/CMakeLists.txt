cmake_minimum_required(VERSION 2.8.3)
project(sound_player_server)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    pygame_managed_player
    mongodb_media_server
    message_generation
)

add_service_files(
    FILES
    PlaySoundService.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package()

#############
## Install ##
#############

install(PROGRAMS
  scripts/sound_player.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
