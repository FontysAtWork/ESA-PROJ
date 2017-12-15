cmake_minimum_required(VERSION 2.8.8)

add_compile_options(-std=c++11)
project(keepout_map_server_msg)

find_package(catkin REQUIRED COMPONENTS roscpp message_generation)
include_directories(${catkin_INCLUDE_DIRS})

if(NOT catkin_FOUND)
  message(FATAL_ERROR "find_package(catkin) failed. catkin was neither found in the workspace nor in the CMAKE_PREFIX_PATH. One reason may be that no ROS setup.sh was sourced before.")
endif()

add_message_files(FILES Line.msg Square.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(LIBRARIES keepout_map_server_msg)

add_library(keepout_map_server_msg SHARED /dev/null)

set_target_properties(keepout_map_server_msg PROPERTIES LINKER_LANGUAGE CXX)

install(TARGETS keepout_map_server_msg
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)