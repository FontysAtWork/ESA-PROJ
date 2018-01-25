cmake_minimum_required(VERSION 2.8.8)
add_compile_options(-std=c++11)
project(nogo_zone_map_server)

find_package(nogo_zone_map_server_msg REQUIRED)

if(NOT nogo_zone_map_server_msg_FOUND)
  message(FATAL_ERROR "find_package(nogo_zone_map_server_msg) failed.")
endif()

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs nogo_zone_map_server_msg)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES nogo_zone_map_server
  CATKIN_DEPENDS roscpp std_msgs message_runtime nogo_zone_map_server_msg
)

add_executable(nogo_zone_map_server src/nogo_zone_map_server.cpp src/map.cpp)

add_dependencies(nogo_zone_map_server ${catkin_EXPORTED_TARGETS})

target_link_libraries(nogo_zone_map_server ${catkin_LIBRARIES})

install(TARGETS nogo_zone_map_server RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})