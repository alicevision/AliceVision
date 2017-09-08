add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(openMVG_sample_robustFundamental robust_fundamental.cpp)

target_link_libraries(openMVG_sample_robustFundamental
  openMVG_image
  openMVG_multiview
  openMVG_features
  openMVG_matching
  stlplus
  vlsift
)

set_property(TARGET openMVG_sample_robustFundamental
  PROPERTY FOLDER AliceVision/Samples
)
