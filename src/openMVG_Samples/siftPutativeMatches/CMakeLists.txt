add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(aliceVision_sample_siftPutative siftmatch.cpp)

target_link_libraries(aliceVision_sample_siftPutative
  aliceVision_image
  aliceVision_features
  aliceVision_matching
  vlsift
  stlplus
)

set_property(TARGET aliceVision_sample_siftPutative
  PROPERTY FOLDER AliceVision/Samples
)
