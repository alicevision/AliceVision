add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(aliceVision_samples_describeAndMatch main_describeAndMatch.cpp)

target_link_libraries(aliceVision_samples_describeAndMatch
  aliceVision_system
  aliceVision_image
  aliceVision_feature
  aliceVision_matching
  vlsift
  ${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_samples_describeAndMatch
  PROPERTY FOLDER Samples
)
