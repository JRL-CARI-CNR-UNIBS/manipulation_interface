cmake_minimum_required(VERSION 3.0.2)
project(manipulation_interface_mongo)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
	std_msgs
)

## Generate services in the 'srv' folder
 add_service_files(
	 FILES
         LoadParam.srv
	 SaveParam.srv
         GetDbNames.srv
         ChangeDb.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
	 DEPENDENCIES
	 std_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES manipulation_interface_mongo
#  CATKIN_DEPENDS message_generation rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)





