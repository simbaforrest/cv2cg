# this CMakeLists file automatically set app_name as current_folder_name
# so no need to manually modify this file for setting app_name
#	simbaforrest
macro(AutoAppConfig)
	INCLUDE_DIRECTORIES(
		${CMAKE_SOURCE_DIR}/include
		${CMAKE_CURRENT_BINARY_DIR}
	)

	# To use all *.cpp, *.hpp files
	FILE(GLOB ALL_HEADER_FILES
		*.hpp
		*.h
		*.hxx
	)
	FILE(GLOB CPP_FILES
		*.cpp
	)

	get_filename_component(CURRENT_FOLDER_PATH ${CMAKE_CURRENT_LIST_FILE} PATH)
	get_filename_component(CURRENT_FOLDER_NAME ${CURRENT_FOLDER_PATH} NAME)
	SET(APP_NAME ${CURRENT_FOLDER_NAME})

	SET(lib_SRCS
		${CPP_FILES}
		${ALL_HEADER_FILES}
	)

	ADD_EXECUTABLE(${APP_NAME} ${lib_SRCS})
	TARGET_LINK_LIBRARIES(${APP_NAME} ${CV2CG_3RDPARTY_LINKEROPTION})

	INSTALL(TARGETS ${APP_NAME} DESTINATION bin)

	message(STATUS "Finished configuring application: ${APP_NAME}")
endmacro(AutoAppConfig)