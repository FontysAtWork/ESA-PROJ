cmake_minimum_required(VERSION 2.8.8)
add_compile_options(-std=c++11 -Wall)
project(task_handler)

file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

find_package(catkin REQUIRED COMPONENTS roscpp atwork_ros_msgs actionserver_tutorial actionlib)
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
 
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp atwork_ros_msgs actionserver_tutorial actionlib
)

add_executable(task_handler ${SOURCES})

target_link_libraries(task_handler ${catkin_LIBRARIES} task_exec_lib)
# 
add_dependencies(task_handler ${actionserver_tutorial_EXPORTED_TARGETS})
add_dependencies(task_handler ${catkin_EXPORTED_TARGETS})
add_dependencies(task_handler atwork_ros_msgs_generate_messages_cpp )
# add_dependencies(task_handler task_exec_lib)
# 
 install(TARGETS task_handler RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
