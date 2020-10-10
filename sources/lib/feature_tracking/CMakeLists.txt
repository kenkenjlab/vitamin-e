##
## Copyright 2020 kenken. All rights reserved.
##

#### CMake Settings ####

cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
add_definitions(-DUNICODE -D_UNICODE)

#### Subproject Settings ####

set(SUBPROJECT_NAME feature_tracking)

file(GLOB SOURCE_FILES src/*.c src/*.cpp src/*.h src/*.hpp src/*.hxx include/*.h include/*.hpp include/*.hxx)

add_library(${SUBPROJECT_NAME} STATIC ${SOURCE_FILES})

target_include_directories(${SUBPROJECT_NAME} PUBLIC
	../${SUBPROJECT_NAME}/include
	../common/include
	../types/include
	${OpenCV_INCLUDE_DIRS})

target_link_libraries(${SUBPROJECT_NAME}
	types
	${OpenCV_LIBS})

target_compile_features(${SUBPROJECT_NAME} PUBLIC cxx_std_11)

set_target_properties(${SUBPROJECT_NAME} PROPERTIES FOLDER "lib")


#### Test Settings ####

if(BUILD_TEST)
	set(SUBPROJECT_TEST_NAME test_${SUBPROJECT_NAME})

	file(GLOB TEST_FILES test/*.c test/*.cpp test/*.h test/*.hpp test/*.hxx)
	
	add_executable(${SUBPROJECT_TEST_NAME} ${TEST_FILES})

	target_include_directories(${SUBPROJECT_TEST_NAME} PUBLIC
		../${SUBPROJECT_NAME}/include
		../common/include
		../types/include
		../video/include
		../visualization/include
		${OpenCV_INCLUDE_DIRS})

	target_link_libraries(${SUBPROJECT_TEST_NAME}
		types
		video
		${SUBPROJECT_NAME}
		visualization
		${OpenCV_LIBS})

	target_compile_features(${SUBPROJECT_TEST_NAME} PUBLIC cxx_std_11)

	set_target_properties(${SUBPROJECT_TEST_NAME} PROPERTIES FOLDER "lib_test")
	
endif()