cmake_minimum_required(VERSION 2.8.3)
project(manipulator)

find_package(catkin REQUIRED
	std_msgs
	message_generation
)


 add_service_files(
   FILES
   #manipulator_S.srv
   isManipulable.srv
 )

generate_messages(
  DEPENDENCIES
   std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES manipulator
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
