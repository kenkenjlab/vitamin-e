##
## Copyright 2020 kenken. All rights reserved.
##

#### CMake Settings ####

cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
add_definitions(-DUNICODE -D_UNICODE)

#### Subproject Settings ####

set(SUBPROJECT_NAME slam)

file(GLOB SOURCE_FILES src/*.c src/*.cpp src/*.h src/*.hpp src/*.hxx)

add_executable(${SUBPROJECT_NAME} ${SOURCE_FILES})

target_include_directories(${SUBPROJECT_NAME} PUBLIC
	../../lib/vitamin_e/include
	../../lib/calibration/include
	../../lib/video/include
	../../lib/visualization/include)

target_link_libraries(${SUBPROJECT_NAME}
	vitamin_e
	calibration
	video
	visualization)

target_compile_features(${SUBPROJECT_NAME} PUBLIC cxx_std_11)

set_target_properties(${SUBPROJECT_NAME} PROPERTIES FOLDER "app")