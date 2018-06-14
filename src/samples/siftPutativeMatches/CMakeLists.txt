add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(aliceVision_samples_siftMatching main_siftMatching.cpp)

target_link_libraries(aliceVision_samples_siftMatching
  aliceVision_image
  aliceVision_feature
  aliceVision_matching
  vlsift
)

set_property(TARGET aliceVision_samples_siftMatching
  PROPERTY FOLDER Samples
)
