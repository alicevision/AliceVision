add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(aliceVision_sample_kvld kvld_filter.cpp)

target_link_libraries(aliceVision_sample_kvld
  aliceVision_image
  aliceVision_multiview
  aliceVision_kvld
  aliceVision_features
  aliceVision_matching
  stlplus
  vlsift)

set_property(TARGET aliceVision_sample_kvld
  PROPERTY FOLDER AliceVision/Samples
)
