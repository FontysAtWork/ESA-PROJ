cmake_minimum_required(VERSION 2.8.8)
add_compile_options(-std=c++11)
project(task_executor)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -DHAVE_LIBCRYPTO")

file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

file(GLOB_RECURSE ACTIONS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS action/*.action)

find_package(catkin REQUIRED COMPONENTS roscpp actionlib_msgs actionlib std_msgs message_generation atwork_ros_msgs atwork_pb_msgs geometry_msgs nav_lib)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_action_files(DIRECTORY . FILES ${ACTIONS})

generate_messages(DEPENDENCIES std_msgs actionlib_msgs atwork_ros_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp actionlib_msgs atwork_ros_msgs nav_lib
)

add_library(task_exec_lib src/LocationIdentifier.cpp src/ObjectIdentifier.cpp)



add_executable(task_executor ${SOURCES})
#add_executable(fibonacci_client src/fibonacci_client.cpp)

add_dependencies(task_executor actionserver_generate_messages_cpp)
add_dependencies(task_exec_lib actionserver_generate_messages_cpp)
#add_dependencies(fibonacci_client actionserver_generate_messages_cpp)


#add_dependencies(task_executor task_handler fibonacci_client)
add_dependencies(task_executor ${catkin_EXPORTED_TARGETS})
add_dependencies(task_exec_lib ${catkin_EXPORTED_TARGETS})

add_dependencies(task_executor ${task_executor_EXPORTED_TARGETS})
add_dependencies(task_exec_lib ${task_executor_EXPORTED_TARGETS})

add_dependencies(task_executor atwork_ros_msgs_generate_messages_cpp nav_lib)
add_dependencies(task_exec_lib atwork_ros_msgs_generate_messages_cpp)


target_link_libraries(task_executor  ${catkin_LIBRARIES})
#target_link_libraries(fibonacci_client  ${catkin_LIBRARIES})

install(
	TARGETS task_executor
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


