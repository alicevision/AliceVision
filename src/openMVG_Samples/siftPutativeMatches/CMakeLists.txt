add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(openMVG_sample_siftPutative siftmatch.cpp)

target_link_libraries(openMVG_sample_siftPutative
  openMVG_image
  openMVG_features
  openMVG_matching
  vlsift
  stlplus
)

set_property(TARGET openMVG_sample_siftPutative
  PROPERTY FOLDER OpenMVG/Samples
)
