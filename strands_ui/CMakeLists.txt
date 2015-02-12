cmake_minimum_required(VERSION 2.8.3)
project(strands_ui)

find_package(catkin REQUIRED COMPONENTS
  rospy
)



catkin_python_setup()



add_message_files(
  FILES
  SoundPriorities.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs  
)


catkin_package()




install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
