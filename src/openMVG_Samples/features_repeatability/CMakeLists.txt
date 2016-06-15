find_package(Popsift)

add_executable(openMVG_sample_main_repeatability_dataset main_repeatability_dataset.cpp)
target_link_libraries(openMVG_sample_main_repeatability_dataset
  openMVG_image
  openMVG_features
  openMVG_matching
  openMVG_system
  openMVG_multiview
  vlsift
  ${POPSIFT_LIBRARIES}
  stlplus
  ${CUDA_LIBRARIES}
  ${CUDA_CUDADEVRT_LIBRARY}
  ${OpenCV_LIBS}
)
target_include_directories(openMVG_sample_main_repeatability_dataset PUBLIC
  ${POPSIFT_INCLUDE_DIRS}
)

set_property(TARGET openMVG_sample_main_repeatability_dataset PROPERTY FOLDER OpenMVG/Samples)

