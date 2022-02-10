cmake_minimum_required(VERSION 3.0.2)
project(nlosExclusion)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  GNSS_Raw.msg
  GNSS_Raw_Array.msg
  DOP.msg
  Satellite_Info.msg
  exclusionSatNum.msg
  Error.msg
)

generate_messages(
  DEPENDENCIES 
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
    rospy
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


