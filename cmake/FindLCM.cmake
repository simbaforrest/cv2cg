#############################################
#  FindLCM.cmake
#  Need setup environment variable LCM_DIR
#  so that ${LCM_DIR}/lcm/lcm-cpp.hpp can be
#  found
#  If successful will set LCM_LIBRARY and
#  LCM_INCLUDE_DIR
#  Author: simbaforrest at gmail dot com
#############################################

IF(NOT LCM_DIR)
	SET(LCM_DIR "$ENV{LCM_DIR}" CACHE PATH "$ENV{LCM_DIR}")
ENDIF()
IF(LCM_DIR STREQUAL "")
  # not set with env var, abort
  MESSAGE(FATAL_ERROR "Please set environment variable LCM_DIR!")
ENDIF(LCM_DIR STREQUAL "")

IF(NOT EXISTS ${LCM_DIR}/lcm/lcm-cpp.hpp)
  # no lcm-cpp.hpp file, abort
  MESSAGE(FATAL_ERROR "LCM_DIR=" ${LCM_DIR} "is not a valid LCM library folder!")
ENDIF(NOT EXISTS ${LCM_DIR}/lcm/lcm-cpp.hpp)

MESSAGE(STATUS "found LCM_DIR=" ${LCM_DIR})

#############################################
# No more environment variable settings

IF(WIN32)
	FIND_PATH(LCM_INCLUDE_DIR lcm/lcm-cpp.hpp
	  ${LCM_DIR}
	)
	FIND_LIBRARY(LCM_LIBRARY lcm
	  ${LCM_DIR}/WinSpecific/release
	)
ELSE(WIN32)
	#TODO
	MESSAGE(FATAL_ERROR "FindLCM.cmake not implemented for non-Windows OS yet!")
ENDIF(WIN32)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(LCM	DEFAULT_MSG
										LCM_INCLUDE_DIR
										LCM_LIBRARY)