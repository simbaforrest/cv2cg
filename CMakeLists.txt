
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

# Avoid repeating the IF statement in the ENDIF and ELSE
SET(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true)

########### Project name ###########
SET(PACKAGE cv2cg)
PROJECT(${PACKAGE})
SET(VERSION_MAJOR "0")
SET(VERSION_MINOR "1")
SET(VERSION_PATCH "0")
SET(VERSION "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}")

########### Main global variables ###########
IF(NOT CMAKE_BUILD_TYPE)
 SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel." FORCE)
ENDIF()

IF(UNIX AND NOT WIN32)
 SET(CMAKE_CXX_FLAGS "-Wall -Wextra -Wno-unused-parameter")		# Enable lots of warning flags
ELSE()
 SET(CMAKE_CXX_FLAGS "-Wall -Wsign-promo -fexceptions -fident -mthreads")	# Refer to Stellarium. Flags used on purpose
 SET(CMAKE_EXE_LINKER_FLAGS "-Wsign-promo -fexceptions -fident -mthreads -mwindows --large-address-aware")
ENDIF()

########### Find packages ###########
#SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake/Modules/)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(osg REQUIRED)
FIND_PACKAGE(OpenThreads REQUIRED)
FIND_PACKAGE(osgDB REQUIRED)
FIND_PACKAGE(osgGA REQUIRED)
FIND_PACKAGE(osgText REQUIRED)
FIND_PACKAGE(osgViewer REQUIRED)
FIND_PACKAGE(osgUtil REQUIRED)

########### uninstall files ###############
CONFIGURE_FILE(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)

ADD_CUSTOM_TARGET(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")
  
########### Packaging info for CPack ###########
INCLUDE(InstallRequiredSystemLibraries)

SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "cv2cg is a lightweight library for computer vision and computer graphics interactions.")
SET(CPACK_PACKAGE_VENDOR "Chen Feng@Umich")
SET(CPACK_GENERATOR "DEB")
SET(CPACK_PACKAGE_CONTACT "Chen Feng <simbaforrest@gmail.com>")
SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "Chen Feng")
SET(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/README")
SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/COPYING")
SET(CPACK_PACKAGE_VERSION_MAJOR ${VERSION_MAJOR})
SET(CPACK_PACKAGE_VERSION_MINOR ${VERSION_MINOR})
SET(CPACK_PACKAGE_VERSION_PATCH ${VERSION_PATCH})
SET(CPACK_SOURCE_IGNORE_FILES "/CVS/" "/.svn/" "/.git/" "/.hg/" "builds/" "~$" "\\\\.swp$" "\\\\.#" "/#")
INCLUDE(CPack)

########### Subdirectories ###############
ADD_SUBDIRECTORY(data)
ADD_SUBDIRECTORY(include)
ADD_SUBDIRECTORY(application)
