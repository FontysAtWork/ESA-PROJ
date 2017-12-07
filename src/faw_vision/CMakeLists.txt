cmake_minimum_required(VERSION 2.8.3)
project(faw_vision)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  #roscpp
  #std_msgs
  #tf
  #message_generation
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

