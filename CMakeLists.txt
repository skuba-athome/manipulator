cmake_minimum_required(VERSION 2.8.3)
project(manipulator)

find_package(catkin REQUIRED
	std_msgs
	message_generation
	roscpp
	sensor_msgs
  	shape_msgs
	geometry_msgs
	tf
	moveit_core 
  	moveit_ros_planning
	moveit_ros_planning_interface
	moveit_msgs
)


 add_service_files(
   FILES
   #manipulator_S.srv
   isManipulable.srv
   ManipulateAction.srv
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

link_directories(${catkin_LIBRARY_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(pick_place_tutorial nodes/pick_place_tutorial.cpp)
target_link_libraries(pick_place_tutorial ${catkin_LIBRARIES})


install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
