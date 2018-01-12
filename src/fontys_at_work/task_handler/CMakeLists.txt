cmake_minimum_required(VERSION 2.8.8)
add_compile_options(-std=c++11)
project(task_handler)
# 
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -DHAVE_LIBCRYPTO")
# 
# file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)
# 
# find_package(Boost REQUIRED COMPONENTS system filesystem thread)
# 
find_package(catkin REQUIRED COMPONENTS actionserver_tutorial roscpp actionlib_msgs protobuf_comm atwork_pb_msgs atwork_ros_msgs std_msgs geometry_msgs message_generation actionlib)
#find_package(catkin REQUIRED COMPONENTS actionserver_tutorial roscpp)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
# 
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp actionserver_tutorial
  task_exec_lib
)

# catkin_package(
#   INCLUDE_DIRS include
#   CATKIN_DEPENDS roscpp protobuf_comm atwork_pb_msgs atwork_ros_msgs actionserver_tutorial actionlib_msgs
# )
# 
# add_executable(task_handler ${SOURCES})
# 
# target_link_libraries(task_handler ${catkin_LIBRARIES} ${Boost_LIBRARIES})
# target_link_libraries(task_handler yaml)
# 
# add_dependencies(task_handler ${actionserver_tutorial_EXPORTED_TARGETS})
# add_dependencies(task_handler atwork_ros_msgs_generate_messages_cpp)
# add_dependencies(task_handler ${catkin_EXPORTED_TARGETS})
# add_dependencies(task_handler actionserver_tutorial_generate_messages_cpp)
# 
# 
# 
# install(TARGETS task_handler RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


add_executable(task_handler src/task_handler.cpp)# src/ObjectIdentifier.cpp src/LocationIdentifier.cpp)

target_link_libraries(task_handler ${catkin_LIBRARIES})

add_dependencies(task_handler ${actionserver_tutorial_EXPORTED_TARGETS})
add_dependencies(task_handler ${catkin_EXPORTED_TARGETS})
add_dependencies(task_handler atwork_ros_msgs_generate_messages_cpp)
add_dependencies(task_handler actionserver_tutorial_generate_messages_cpp)

install(TARGETS task_handler RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
