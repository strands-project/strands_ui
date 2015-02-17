cmake_minimum_required(VERSION 2.8.3)
project(pygame_managed_player)

find_package(catkin REQUIRED COMPONENTS
	roscpp rospy std_msgs message_generation
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


catkin_package(
	CATKIN_DEPENDS message_runtime
)


