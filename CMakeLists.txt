cmake_minimum_required(VERSION 2.8.3)
project(aaf_music_player)

find_package(catkin REQUIRED COMPONENTS
    genmsg
    actionlib
    actionlib_msgs
    mary_tts
    rospy
)

catkin_python_setup()

add_service_files(
    FILES
    MusicPlayerService.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package()

#############
## Install ##
#############
#install(PROGRAMS
#    scripts/toggle_walking_group_recovery_server.py
#    src/walking_group_recovery/walking_group_recovery_state.py
#    src/walking_group_recovery/walking_group_nav_states.py
#    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
