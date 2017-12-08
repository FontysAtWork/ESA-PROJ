cmake_minimum_required(VERSION 2.8.8)
add_compile_options(-std=c++11)
project(keepout_map_server)

find_package(keepout_map_server_msg REQUIRED)

if(NOT keepout_map_server_msg_FOUND)
  message(FATAL_ERROR "find_package(keepout_map_server_msg) failed.")
endif()

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs keepout_map_server_msg)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES keepout_map_server
  CATKIN_DEPENDS roscpp std_msgs message_runtime keepout_map_server_msg
)

add_executable(keepout_map_server src/keepout_map_server.cpp src/map.cpp)

add_dependencies(keepout_map_server ${catkin_EXPORTED_TARGETS})

target_link_libraries(keepout_map_server ${catkin_LIBRARIES})

install(TARGETS keepout_map_server RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})