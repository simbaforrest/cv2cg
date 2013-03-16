
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

# Avoid repeating the IF statement in the ENDIF and ELSE
SET(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true)

########### Project name ###########
SET(PACKAGE cv2cg)
PROJECT(${PACKAGE})
SET(VERSION_MAJOR "1")
SET(VERSION_MINOR "0")
SET(VERSION_PATCH "0")
SET(VERSION "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}")

########### Main global variables ###########
IF(NOT CMAKE_BUILD_TYPE)
 SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel." FORCE)
ENDIF()

IF(UNIX AND NOT WIN32)
 SET(CMAKE_CXX_FLAGS "-Wall -Wextra -Wno-unused-parameter -fpermissive")		# Enable lots of warning flags
ELSE()
	SET(CMAKE_CXX_FLAGS "/fp:precise /fp:except /bigobj /Zm1000 /EHsc /DWIN32")	# Refer to Stellarium. Flags used on purpose
ENDIF()

########### Find packages ###########
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake/)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(OpenSceneGraph 2.0.0 REQUIRED
	osg
	osgDB
	osgGA
	osgUtil
	osgText
	osgViewer
	OpenThreads
)
FIND_PACKAGE(Eigen3 REQUIRED)

#define ESM_LIB, ESM_HEADER
IF(UNIX AND NOT WIN32)
 find_library(ESM_LIB ESMlib.a PATHES ${CMAKE_SOURCE_DIR}/include/esm/ NO_DEFAULT_PATH)
 find_file(ESM_HEADER ESMlibry.h PATHES ${CMAKE_SOURCE_DIR}/include/esm/ NO_DEFAULT_PATH)
ELSE()
 find_library(ESM_LIB ESMlibry.lib PATHES ${CMAKE_SOURCE_DIR}/include/esm/ NO_DEFAULT_PATH)
 find_file(ESM_DLL ESMlibry.dll PATHES ${CMAKE_SOURCE_DIR}/include/esm/ NO_DEFAULT_PATH)
 find_file(ESM_HEADER ESMlibry.h PATHES ${CMAKE_SOURCE_DIR}/include/esm/ NO_DEFAULT_PATH)
ENDIF()

########### uninstall files ###############
CONFIGURE_FILE(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)

ADD_CUSTOM_TARGET(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")

########### Subdirectories ###############
ADD_SUBDIRECTORY(application)
ADD_SUBDIRECTORY(data)
