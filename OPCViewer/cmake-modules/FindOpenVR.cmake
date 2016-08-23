set(OPENVR_INCLUDE_DIR ../openvr/headers)
set(OPENVR_LIB_DIR ../openvr/lib/linux64/libopenvr_api.so)

IF (WIN32)
	FIND_PATH(OPENVR_INCLUDE_PATH openvr/headers
			NAMES openvr.h openvr_capi.h openvr_driver.h
			PATHS 	${CMAKE_SOURCE_DIR}/includes
					$ENV{PROGRAMFILES}/openvr/headers
					${OPENVR_ROOT_DIR}/headers
					$ENV{OPENVR_ROOT_DIR}/headers
			DOC "The directory where openvr.h resides")
	IF (NV_SYSTEM_PROCESSOR STREQUAL "AMD64")
		FIND_LIBRARY(OPENVR_LIBRARY
				NAMES openvr OpenVR openvr_api.lib
				PATHS	$ENV{PROGRAMFILES}/openvr/lib/win64
						$ENV{OPENVR_ROOT_DIR}/lib/win64
						${PROJECT_SOURCE_DIR}/src/openvr/lib/win64
				DOC "The OpenVR library (64-bit)"
				)
	ELSE(NV_SYSTEM_PROCESSOR STREQUAL "AMD64")
		FIND_LIBRARY( OPENVR_LIBRARY
				NAMES openvr OpenVR openvr_api.lib
				PATHS	$ENV{PROGRAMFILES}/openvr/lib/win32
						$ENV{OPENVR_ROOT_DIR}/lib/win32
						${PROJECT_SOURCE_DIR}/src/openvr/lib/win32
				DOC "The OpenVR library"
				)
	ENDIF(NV_SYSTEM_PROCESSOR STREQUAL "AMD64")
ELSE (WIN32)
	FIND_PATH(OPENVR_INCLUDE_PATH openvr.h
				/usr/include
				/usr/local/include
				/sw/include
				/opt/local/include
				$ENV{OPENVR_ROOT_DIR}/headers
				DOC "The directory where openvr.h resides")
	FIND_LIBRARY(OPENVR_LIBRARY
					NAMES openvr.h
					PATHS
					/usr/lib64
					/usr/lib
					/usr/local/lib64
					/usr/local/lib
					/sw/lib
					/opt/local/lib
					$ENV{OPENVR_ROOT_DIR}/lib
					DOC "The OpenVR library")
ENDIF (WIN32)

SET(OPENVR_FOUND "NO")
IF (OPENVR_INCLUDE_PATH AND OPENVR_LIBRARY)
	SET(OPENVR_LIBRARIES ${OPENVR_LIBRARY})
	SET(OPENVR_FOUND "YES")
ENDIF (OPENVR_INCLUDE_PATH AND OPENVR_LIBRARY)
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OPENVR DEFAULT_MSG OPENVR_LIBRARY OPENVR_INCLUDE_PATH)