CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

# Avoid repeating the IF statement in the ENDIF and ELSE
SET(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true)

########### Project name ###########
SET(PACKAGE cv2cg)
PROJECT(${PACKAGE})
SET(VERSION_MAJOR "1")
SET(VERSION_MINOR "2")
SET(VERSION_PATCH "0")
SET(VERSION "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}")

########### Main global variables ###########
IF(NOT CMAKE_BUILD_TYPE)
 SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel." FORCE)
ENDIF()

IF(UNIX AND NOT WIN32)
 SET(CMAKE_CXX_FLAGS "-Wall -Wextra -Wno-unused-parameter -Wno-unknown-pragmas")
ELSE()
	SET(CMAKE_CXX_FLAGS "/fp:precise /fp:except /bigobj /Zm1000 /EHsc /DWIN32")
ENDIF()

########### Find packages ###########
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake/)
include(${CMAKE_SOURCE_DIR}/cmake/SetCV2CGDependencies.cmake)

########### uninstall files ###############
CONFIGURE_FILE(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)

ADD_CUSTOM_TARGET(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")

########### Macros ###############
include(${CMAKE_SOURCE_DIR}/cmake/AutoAppConfig.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/AutoAddSubDir.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/BuildLCMType.cmake)
  
########### Subdirectories ###############
ADD_SUBDIRECTORY(application)
