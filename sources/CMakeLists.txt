##
## Copyright 2020 kenken. All rights reserved.
##

#### CMake Settings ####

cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
add_definitions(-DUNICODE -D_UNICODE)


#### Solution Settings ####

project(VITAMIN-E_proto5 C CXX)

set_property(GLOBAL PROPERTY USE_FOLDERS ON) 
set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "")

if(CMAKE_COMPILER_IS_GNUCC)
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra")
endif()
if(MSVC)
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} /W4")
endif()

#### Options ####

option(BUILD_TEST "Build test project" ON)

#### Libraries ####

find_package(OpenCV REQUIRED
	COMPONENTS core highgui calib3d imgproc features2d viz video
	OPTIONAL_COMPONENTS xfeatures2d rgbd)
if(${OpenCV_VERSION} VERSION_LESS 3.2)
	message(FATAL_ERROR "ERROR: OpenCV 3.2 or later is required. (found version: ${OpenCV_VERSION})")
endif()


#### Includes ####

add_subdirectory(lib)
add_subdirectory(app)