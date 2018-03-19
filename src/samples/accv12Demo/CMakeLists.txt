if(ALICEVISION_HAVE_OPENCV)
    add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    
    add_executable(alicevision_samples_openCVFitting main_openCVFitting.cpp)
    
    target_link_libraries(alicevision_samples_openCVFitting 
		${OpenCV_LIBS} 
    ${Boost_FILESYSTEM_LIBRARIES}
		aliceVision_multiview
    ) 
    
    set_property(TARGET alicevision_samples_openCVFitting
		PROPERTY FOLDER AliceVision/Samples
	)
    
    install(TARGETS alicevision_samples_openCVFitting 
		DESTINATION bin/
    )
endif()


