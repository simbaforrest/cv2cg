macro(BuildLCMType lcm_type_name)
	MESSAGE(STATUS "Generating C++/Python/Java files for " ${lcm_type_name} ".lcm")
	SET(LCM_GEN_CPP "lcm-gen")
	EXECUTE_PROCESS(
		COMMAND ${LCM_GEN_CPP} -x ${lcm_type_name}.lcm
		COMMAND ${LCM_GEN_CPP} -p ${lcm_type_name}.lcm
		COMMAND ${LCM_GEN_CPP} -j ${lcm_type_name}.lcm
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	)

	SET(LCM_JAR ${LCM_DIR}/lcm-java/lcm.jar)
	IF(EXISTS ${LCM_JAR})
		IF(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${lcm_type_name}.jar)
			MESSAGE(STATUS "${lcm_type_name}.jar exist, no need to build")
		ELSE()
			MESSAGE(STATUS "JAVA version:")
			EXECUTE_PROCESS(
				COMMAND cmd /c javac -version
			)
			EXECUTE_PROCESS( #build ${lcm_type_name}.jar
				COMMAND javac -cp ${LCM_JAR} ${lcm_type_name}/*.java
				WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
			)
			EXECUTE_PROCESS( #build ${lcm_type_name}.jar
				COMMAND jar cf ${CMAKE_CURRENT_SOURCE_DIR}/${lcm_type_name}.jar ${lcm_type_name}/*.class
				WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
			)
			EXECUTE_PROCESS( #clean up, this MUST be done ONLY AFTER jar is generated, otherwise the jar file will be invalid
				COMMAND cmd /c del *.class
				WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${lcm_type_name}
			)
			MESSAGE(STATUS "Generated Jar file for " ${lcm_type_name} ".")
		ENDIF()
	ELSE()
		MESSAGE(WARNING "Fail to find LCM_JAR=" ${LCM_JAR} ", so no " ${lcm_type_name} ".jar is generated!")
	ENDIF()
endmacro()