IF(USE_LCM) #only build this app if LCM is enabled
	FILE(GLOB LCM_TYPE_FILES
		*.lcm
	)
	FOREACH(lcm_type_file ${LCM_TYPE_FILES})
		get_filename_component(lcm_type_name ${lcm_type_file} NAME_WE)
		BuildLCMType(${lcm_type_name})
	ENDFOREACH()

	IF(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/apriltag_lcm/TagPoseArray_t.hpp)
		MESSAGE(STATUS "lcm-gen of apriltag_lcm.lcm done!")
		AutoAppConfig()
		# installing an example cfg file...
		INSTALL(FILES AprilTagFinderLCM.cfg DESTINATION bin)
		FILE(GLOB JAR_FILES
			*.jar
		)
		FOREACH(jar_file ${JAR_FILES})
			get_filename_component(jar_file_name ${jar_file} NAME)
			INSTALL(FILES ${jar_file_name} DESTINATION bin)
		ENDFOREACH()

		FILE(GLOB M_FILES
			*.m
		)
		FOREACH(m_file ${M_FILES})
			get_filename_component(m_file_name ${m_file} NAME)
			INSTALL(FILES ${m_file_name} DESTINATION bin)
		ENDFOREACH()
		INSTALL(FILES ${LCM_JAR} DESTINATION bin)
	ELSE()
		MESSAGE(WARNING "lcm-gen of apriltag_lcm.lcm failed!")
	ENDIF()
ENDIF()