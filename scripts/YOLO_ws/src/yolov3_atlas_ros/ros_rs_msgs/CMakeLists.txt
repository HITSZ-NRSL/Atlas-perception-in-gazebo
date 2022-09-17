cmake_minimum_required(VERSION 2.8.3)
project(ros_rs_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
message_generation
std_msgs
roscpp
)


add_message_files(DIRECTORY msg
  FILES 
  DetectionMessage.msg
  DetectionMessages.msg)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
        std_msgs
)
