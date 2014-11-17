cmake_minimum_required(VERSION 2.8.3)
project(manipulator)

find_package(catkin REQUIRED
	std_msgs
)


 add_service_files(
   FILES
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