cmake_minimum_required(VERSION 3.5)

project(OOPCV)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set (CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} /W4)
 
message (cxx Flags: ${CMAKE_CXX_FLAGS})

add_subdirectory(ros_lib)
add_subdirectory(OPCViewer)
