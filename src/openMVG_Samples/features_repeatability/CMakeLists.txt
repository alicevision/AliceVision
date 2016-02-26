
# look for popsift
find_package(popsift)
include_directories(${POPSIFT_INCLUDE_DIRS})

ADD_EXECUTABLE(openMVG_sample_main_repeatability_dataset main_repeatability_dataset.cpp)
TARGET_LINK_LIBRARIES(openMVG_sample_main_repeatability_dataset
  openMVG_image
  openMVG_features
  openMVG_matching
  openMVG_system
  openMVG_multiview
  vlsift
  popsift
  stlplus)

SET_PROPERTY(TARGET openMVG_sample_main_repeatability_dataset PROPERTY FOLDER OpenMVG/Samples)
