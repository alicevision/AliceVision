add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(aliceVision_samples_demoAKAZE main_demoAKAZE.cpp)

target_link_libraries(aliceVision_samples_demoAKAZE
  aliceVision_image
  aliceVision_feature
  stlplus
  ${ALICEVISION_LIBRARY_DEPENDENCIES}
)

set_property(TARGET aliceVision_samples_demoAKAZE 
	PROPERTY FOLDER AliceVision/Samples
)

add_executable( aliceVision_samples_computeAKAZE main_computeAKAZE.cpp)

target_link_libraries( aliceVision_samples_computeAKAZE
  aliceVision_image
  aliceVision_feature
  aliceVision_system
  stlplus
  ${ALICEVISION_LIBRARY_DEPENDENCIES}
)
  
set_property(TARGET aliceVision_samples_computeAKAZE 
	PROPERTY FOLDER AliceVision/Samples
)
