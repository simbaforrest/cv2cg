#define ESM_LIB, ESM_HEADER
IF(UNIX AND NOT WIN32)
 find_library(ESM_LIB ESMlib.a PATHES ${CMAKE_SOURCE_DIR}/include/esm/ NO_DEFAULT_PATH)
 find_file(ESM_HEADER ESMlibry.h PATHES ${CMAKE_SOURCE_DIR}/include/esm/ NO_DEFAULT_PATH)
ELSE()
 find_library(ESM_LIB ESMlibry.lib PATHES ${CMAKE_SOURCE_DIR}/include/esm/ NO_DEFAULT_PATH)
 find_file(ESM_DLL ESMlibry.dll PATHES ${CMAKE_SOURCE_DIR}/include/esm/ NO_DEFAULT_PATH)
 find_file(ESM_HEADER ESMlibry.h PATHES ${CMAKE_SOURCE_DIR}/include/esm/ NO_DEFAULT_PATH)
ENDIF()

#message(${ESM_LIB})
AutoAppConfig()
TARGET_LINK_LIBRARIES(${APP_NAME} ${ESM_LIB})

INSTALL(FILES
	${ESM_DLL}
DESTINATION bin)

# installing an example cfg file also
INSTALL(FILES KEG.cfg DESTINATION bin)