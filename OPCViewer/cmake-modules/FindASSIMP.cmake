# - Try to find Assimp
# Once done, this will define
#
# ASSIMP_FOUND - system has Assimp
# ASSIMP_INCLUDE_DIR - the Assimp include directories
# ASSIMP_LIBRARIES - link these to use Assimp

IF (WIN32)

	FIND_PATH( ASSIMP_INCLUDE_DIR 
			NAMES	assimp/mesh.h
			PATHS 	${CMAKE_SOURCE_DIR}/includes
					$ENV{PROGRAMFILES}/assimp/include
					${ASSIMP_ROOT_DIR}/include
					$ENV{ASSIMP_ROOT_DIR}/include
			DOC "The directory where assimp/mesh.h resides"
	)
	IF (NV_SYSTEM_PROCESSOR STREQUAL "AMD64")
		FIND_LIBRARY(ASSIMP_LIBRARY
				NAMES assimp.lib
				PATHS	$ENV{PROGRAMFILES}/assimp/lib64
						$ENV{ASSIMP_ROOT_DIR}/lib64
						${PROJECT_SOURCE_DIR}/src/assimp/lib64
				DOC "The ASSIMP library (64-bit)"
				)
	ELSE(NV_SYSTEM_PROCESSOR STREQUAL "AMD64")
		FIND_LIBRARY( ASSIMP_LIBRARY
				NAMES assimp.lib
				PATHS	$ENV{PROGRAMFILES}/assimp/lib32
						$ENV{ASSIMP_ROOT_DIR}/lib32
						${PROJECT_SOURCE_DIR}/src/assimp/lib32
				DOC "The ASSIMP library"
				)
	ENDIF(NV_SYSTEM_PROCESSOR STREQUAL "AMD64")

ELSE (WIN32)

	FIND_PATH( ASSIMP_INCLUDE_DIR assimp/mesh.h
		/usr/include
		/usr/local/include
		/opt/local/include
		${CMAKE_SOURCE_DIR}/includes
	)
	FIND_LIBRARY( ASSIMP_LIBRARY assimp
		/usr/lib64
		/usr/lib
		/usr/local/lib
		/opt/local/lib
		${CMAKE_SOURCE_DIR}/lib
	)

ENDIF (WIN32)

IF(ASSIMP_INCLUDE_DIR AND ASSIMP_LIBRARY)
	SET( ASSIMP_FOUND TRUE )
	SET( ASSIMP_LIBRARIES ${ASSIMP_LIBRARY} )
ENDIF(ASSIMP_INCLUDE_DIR AND ASSIMP_LIBRARY)

IF(ASSIMP_FOUND)
	IF(NOT ASSIMP_FIND_QUIETLY)
	MESSAGE(STATUS "Found ASSIMP: ${ASSIMP_LIBRARY}")
	ENDIF(NOT ASSIMP_FIND_QUIETLY)

ELSE(ASSIMP_FOUND)
	IF(ASSIMP_FIND_REQUIRED)
	MESSAGE(FATAL_ERROR "Could not find libASSIMP")
	ENDIF(ASSIMP_FIND_REQUIRED)
	
ENDIF(ASSIMP_FOUND)