cmake_minimum_required(VERSION 3.5)

project(ros_lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake-modules/")

include_directories(.)

file(GLOB ros_lib_SOURCE
	*.cpp
	*.h
)

include_directories(.)

# library
add_library(ros_lib ${ros_lib_SOURCE})
target_include_directories(ros_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})