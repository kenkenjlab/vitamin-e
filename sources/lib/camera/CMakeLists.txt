##
## Copyright 2020 kenken. All rights reserved.
##

#### CMake Settings ####

cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
add_definitions(-DUNICODE -D_UNICODE)

#### Subproject Settings ####

set(SUBPROJECT_NAME camera)

file(GLOB SOURCE_FILES src/*.c src/*.cpp src/*.h src/*.hpp src/*.hxx include/*.h include/*.hpp include/*.hxx)

add_library(${SUBPROJECT_NAME} STATIC ${SOURCE_FILES})

target_include_directories(${SUBPROJECT_NAME} PUBLIC
	../${SUBPROJECT_NAME}/include
	../common/include
	../types/include
	../utils/include
	${OpenCV_INCLUDE_DIRS})

target_link_libraries(${SUBPROJECT_NAME}
	utils)

target_compile_features(${SUBPROJECT_NAME} PUBLIC cxx_std_11)

set_target_properties(${SUBPROJECT_NAME} PROPERTIES FOLDER "lib")