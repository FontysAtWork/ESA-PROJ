cmake_minimum_required (VERSION 2.8)

# This is the name of the project
PROJECT( cppInterface  )

add_definitions(-std=c++11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# This is for recursively searching for other directories.
# add_subdirectory (DirectoryName)

FIND_PACKAGE( OpenCV REQUIRED )
set(DEPENDENCIES realsense2 ${OpenCV_LIBS} ${LIBUSB1_LIBRARIES})

# sources to include
include_directories (.)
FIND_PACKAGE( realsense2 REQUIRED )
ADD_EXECUTABLE( cppInterface.out  main.cpp)
TARGET_LINK_LIBRARIES( cppInterface.out ${DEPENDENCIES} )
