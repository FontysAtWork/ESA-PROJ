cmake_minimum_required(VERSION 2.8.8)

add_compile_options(-std=c++11)
project(nav_lib)

#include_directories(include ${catkin_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS roscpp)
include_directories(${catkin_INCLUDE_DIRS} include/nav_lib/)

if(NOT catkin_FOUND)
  message(FATAL_ERROR "find_package(catkin) failed. catkin was neither found in the workspace nor in the CMAKE_PREFIX_PATH. One reason may be that no ROS setup.sh was sourced before.")
endif()

#file(GLOB include/YamlLib/*.hpp)
#file(GLOB src/*.cpp)

add_library(nav_lib src/Marker.cpp src/NoGoLine.cpp src/YamlParser.cpp src/YamlWriter.cpp src/MapConfig.cpp)

target_link_libraries(nav_lib ${catkin_LIBRARIES})

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp)

#include_directories(include/YamlLib/)

install(TARGETS nav_lib
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
