cmake_minimum_required(VERSION 2.8.3)
project(faw_vision)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
)

catkin_package (
  #INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS roscpp sensor_msgs tf
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(broadcast_tf_node src/broadcast_tf.cpp)
target_link_libraries(broadcast_tf_node ${catkin_LIBRARIES})
