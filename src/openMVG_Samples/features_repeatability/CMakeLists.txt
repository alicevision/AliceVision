add_executable(openMVG_sample_main_repeatability_dataset main_repeatability_dataset.cpp)
target_link_libraries(openMVG_sample_main_repeatability_dataset
  openMVG_image
  openMVG_features
  openMVG_matching
  openMVG_system
  openMVG_multiview
  vlsift
  PopSift::popsift # ${POPSIFT_LIBRARIES}
  cudaSift::cudasift # ${CUDASIFT_LIBRARIES}
  stlplus
  ${OpenCV_LIBS}
)

set_property(TARGET openMVG_sample_main_repeatability_dataset PROPERTY FOLDER OpenMVG/Samples)

