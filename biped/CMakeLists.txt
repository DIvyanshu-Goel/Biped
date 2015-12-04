cmake_minimum_required(VERSION 2.8.3)
project(biped)

find_package(catkin REQUIRED COMPONENTS
  dynamixel_controllers
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
   FILES
   Motor_States.msg
 )

add_service_files(
   FILES
   Fetch_Motor_Data.srv
 )

generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES biped
  CATKIN_DEPENDS std_msgs message_runtime dynamixel_controllers roscpp rospy 
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  )
