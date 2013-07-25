# This CMakeLists file does nothing but loop over subdirectories
# and call ADD_SUBDIRECTORY
#	simbaforrest

MACRO(SUBDIRLIST result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
  SET(dirlist "")
  FOREACH(child ${children})
    IF(IS_DIRECTORY ${curdir}/${child})
        SET(dirlist ${dirlist} ${child})
    ENDIF()
  ENDFOREACH()
  SET(${result} ${dirlist})
ENDMACRO()

macro(AutoAddSubDir)
SUBDIRLIST(SUBDIRS ${CMAKE_CURRENT_LIST_DIR})
FOREACH(subdir ${SUBDIRS})
	message(STATUS "Add application: ${subdir}")
	ADD_SUBDIRECTORY(${subdir})
ENDFOREACH()
endmacro(AutoAddSubDir)