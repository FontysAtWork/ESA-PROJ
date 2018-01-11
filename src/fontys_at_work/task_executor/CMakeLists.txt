cmake_minimum_required(VERSION 2.8.8)
add_compile_options(-std=c++11)
project(task_executor)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -DHAVE_LIBCRYPTO")

file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

file(GLOB_RECURSE ACTIONS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS action/*.action)

find_package(catkin REQUIRED COMPONENTS roscpp actionlib std_msgs message_generation)
include_directories(include ${catkin_INCLUDE_DIRS})

add_action_files(DIRECTORY . FILES ${ACTIONS})

generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp 
)

add_executable(task_executor SOURCES)

add_dependencies(task_executor task_handler )

target_link_libraries(task_executor  ${catkin_LIBRARIES})


install(TARGETS task_executor RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


