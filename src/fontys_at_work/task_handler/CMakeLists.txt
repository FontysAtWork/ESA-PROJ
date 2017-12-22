cmake_minimum_required(VERSION 2.8.8)
add_compile_options(-std=c++11)
project(task_handler)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -DHAVE_LIBCRYPTO")

file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

find_package(Boost REQUIRED COMPONENTS system filesystem thread)

find_package(catkin REQUIRED COMPONENTS roscpp protobuf_comm atwork_pb_msgs atwork_ros_msgs std_msgs geometry_msgs message_generation)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp protobuf_comm atwork_pb_msgs atwork_ros_msgs
)

add_executable(task_handler ${SOURCES})

add_dependencies(task_handler atwork_ros_msgs_generate_messages_cpp)

target_link_libraries(task_handler ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(task_handler yaml)

install(TARGETS task_handler RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
