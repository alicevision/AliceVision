add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(aliceVision_samples_robustEssentialSpherical main_robustEssentialSpherical.cpp)

target_link_libraries(aliceVision_samples_robustEssentialSpherical
  aliceVision_image
  aliceVision_multiview
  aliceVision_feature
  aliceVision_matching
  aliceVision_system
  aliceVision_sfm
  vlsift
)

set_property(TARGET aliceVision_samples_robustEssentialSpherical 
	PROPERTY FOLDER Samples
)

