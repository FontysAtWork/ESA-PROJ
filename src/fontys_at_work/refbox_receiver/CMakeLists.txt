cmake_minimum_required(VERSION 2.8.8)
add_compile_options(-std=c++11)
project(refbox_receiver)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -DHAVE_LIBCRYPTO")


find_package(Boost REQUIRED COMPONENTS system filesystem thread)

find_package(catkin REQUIRED COMPONENTS roscpp protobuf_comm atwork_pb_msgs atwork_ros_msgs std_msgs geometry_msgs message_generation)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp protobuf_comm atwork_pb_msgs atwork_ros_msgs
)

add_executable(refbox_receiver src/refbox_receiver_node.cpp src/refbox_receiver.cpp)

add_dependencies(refbox_receiver atwork_ros_msgs_generate_messages_cpp)

target_link_libraries(refbox_receiver ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS refbox_receiver RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#install(
#  DIRECTORY ros/include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#)
