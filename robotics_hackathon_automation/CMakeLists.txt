cmake_minimum_required(VERSION 3.0.2)
project(robotics_hackathon_automation)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  message_generation
  roscpp
  rospy
)

add_message_files(
    FILES
    Coordinates.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
     CATKIN_DEPENDS message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
