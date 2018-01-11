cmake_minimum_required(VERSION 2.8.8)
add_compile_options(-std=c++11)
project(actionserver_tutorial)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -DHAVE_LIBCRYPTO")

file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

file(GLOB_RECURSE ACTIONS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS action/*.action)

find_package(catkin REQUIRED COMPONENTS roscpp actionlib_msgs actionlib std_msgs message_generation)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_action_files(DIRECTORY . FILES ${ACTIONS})

generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp actionlib_msgs
)


add_executable(actionserver_tutorial src/fibonacci_server.cpp)
add_executable(fibonacci_client src/fibonacci_client.cpp)

add_dependencies(actionserver_tutorial actionserver_generate_messages_cpp)
add_dependencies(fibonacci_client actionserver_generate_messages_cpp)


add_dependencies(actionserver_tutorial task_handler fibonacci_client)
add_dependencies(actionserver_tutorial ${catkin_EXPORTED_TARGETS})

add_dependencies(actionserver_tutorial ${actionserver_tutorial_EXPORTED_TARGETS})


target_link_libraries(actionserver_tutorial  ${catkin_LIBRARIES})
target_link_libraries(fibonacci_client  ${catkin_LIBRARIES})

install(
	TARGETS actionserver_tutorial
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


