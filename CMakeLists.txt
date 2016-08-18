cmake_minimum_required(VERSION 3.5)

project(OPCViewer)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake-modules/")

# find the required packages
find_package(GLM REQUIRED)
message(STATUS "GLM included at ${GLM_INCLUDE_DIR}")
find_package(GLFW3 REQUIRED)
message(STATUS "Found GLFW3 in ${GLFW3_INCLUDE_DIR}")
find_package(GLEW REQUIRED)
message(STATUS "Found GLEW in ${GLEW_INCLUDE_PATH}")
find_package(OpenVR REQUIRED)
message(STATUS "Found OpenVR in ${OPENVR_INCLUDE_PATH}")

# libraries
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
find_package(OpenGL REQUIRED)
add_definitions(${OPENGL_DEFINITIONS})

IF (WIN32)
	set(LIBS ${GLFW3_LIBRARY} ${OPENGL_gl_LIBRARY} ${GLEW_LIBRARY} ${OPENVR_LIBRARY})
ELSE (WIN32)
	find_package(X11 REQUIRED)
	# note that the order is important for setting the libs
	# use pkg-config --libs $(pkg-config --print-requires --print-requires-private glfw3) in a terminal to confirm
	set(LIBS ${GLFW3_LIBRARY} X11 Xrandr Xinerama Xi Xxf86vm Xcursor GL dl pthread GLEW ${OPENVR_LIBRARY})
ENDIF (WIN32)

include_directories(include ${GLEW_INCLUDE_PATH} ${GLFW3_INCLUDE_DIR} ${OPENVR_INCLUDE_PATH} ${GLM_INCLUDE_DIR})

file(GLOB OPCViewer_SOURCE
	src/*.cpp
	include/*.h
)

# executable
add_executable(OPCViewer ${OPCViewer_SOURCE})
target_link_libraries(OPCViewer ${LIBS})

# shaders
file(GLOB SHADERS
        "shaders/*.fs"
        "shaders/*.vs")

add_custom_command(TARGET OPCViewer POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/shaders
        $<TARGET_FILE_DIR:OPCViewer>/shaders)
