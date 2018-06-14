add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(aliceVision_samples_robustEssentialBA main_robustEssentialBA.cpp)

target_link_libraries(aliceVision_samples_robustEssentialBA
  aliceVision_image
  aliceVision_feature
  aliceVision_matching
  aliceVision_system
  aliceVision_sfm
  vlsift
  ${Boost_FILESYSTEM_LIBRARIES}
)

set_property(TARGET aliceVision_samples_robustEssentialBA
  PROPERTY FOLDER Samples
)
