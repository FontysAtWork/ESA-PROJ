cmake_minimum_required(VERSION 2.8.8)
add_compile_options(-std=c++11)
project(map_marker)

set(ROS_DIST KINETIC)

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/map_marker/*.hpp)
file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

if("$ENV{ROS_DISTRO}" STREQUAL "indigo")

	message(STATUS "Compiling on ROS indigo")

	find_package(catkin REQUIRED COMPONENTS qt_build tf roscpp)
	include_directories(${catkin_INCLUDE_DIRS}) 

	if(NOT catkin_FOUND)
		message(FATAL_ERROR "find_package(catkin) failed. catkin was neither found in the workspace nor in the CMAKE_PREFIX_PATH. One reason may be that no ROS setup.sh was sourced before.")
	endif()

	catkin_package()

	rosbuild_prepare_qt4(QtCore QtGui)

	QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
	QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
	QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

	add_executable(map_marker ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
	target_link_libraries(map_marker ${QT_LIBRARIES} ${catkin_LIBRARIES} yaml)
	install(TARGETS map_marker RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

endif()

if($ENV{ROS_DISTRO} STREQUAL "kinetic")

	message(STATUS "Compiling on ROS kinetic")

	set(CMAKE_AUTOMOC ON)
	set(CMAKE_INCLUDE_CURRENT_DIR ON)

	find_package(catkin REQUIRED COMPONENTS tf roscpp)
	include_directories(${catkin_INCLUDE_DIRS})

	if(NOT catkin_FOUND)
		message(FATAL_ERROR "find_package(catkin) failed. catkin was neither found in the workspace nor in the CMAKE_PREFIX_PATH. One reason may be that no ROS setup.sh was sourced before.")
	endif()

	find_package(Qt5Widgets REQUIRED)
	include_directories(${Qt5Widgets_INCLUDES})

	if(NOT Qt5Widgets_FOUND)
	 	message(FATAL_ERROR "find_package(Qt5Widgets) failed. Qt was not installed. This package is tested with Qt 5.9.2")
	endif()

	find_package(PythonLibs 2.7 REQUIRED)
	include_directories(${PYTHON_INCLUDE_DIRS})

	if(NOT PythonLibs_FOUND)
	 	message(FATAL_ERROR "find_package(PythonLibs) failed. Python was not installed. Try running 'sudo apt-get install pyhton-dev' This package is tested with Python 2.7.12")
	endif()

	find_package(Boost COMPONENTS python REQUIRED)
	include_directories(${Boost_INCLUDE_DIRS})

	if(NOT Boost_FOUND)
	 	message(FATAL_ERROR "find_package(Boost) failed. Boost was not installed. Try running 'sudo apt-get install libboost-all-dev' This package is tested with Boost 1.58.0")
	endif()

	catkin_package()

	add_definitions(${Qt5Widgets_DEFINITIONS})
	set(CMAKE_CXX_FLAGS "${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")

	qt5_add_resources(QT_RESOURCES_CPP ${QT_RESOURCES})
	qt5_wrap_ui(QT_FORMS_HPP ${QT_FORMS})
	qt5_wrap_cpp(QT_MOC_HPP ${QT_MOC})

	add_executable(map_marker ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})

	target_link_libraries(map_marker ${catkin_LIBRARIES})
	target_link_libraries(map_marker Qt5::Widgets)
	target_link_libraries(map_marker ${PYTHON_LIBRARIES})
	target_link_libraries(map_marker ${Boost_LIBRARIES})
	target_link_libraries(map_marker yaml)

	install(TARGETS map_marker RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

endif()