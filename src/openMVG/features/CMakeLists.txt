project(features)

# Headers
set(features_files_headers
  regions.hpp
  akaze/mldb_descriptor.hpp
  akaze/msurf_descriptor.hpp
  akaze/AKAZE.hpp
  sift/SIFT_describer.hpp
  sift/SIFT_vlfeat_describer.hpp
  sift/SIFT_vlfeat_float_describer.hpp
  sift/SIFT.hpp
  image_describer.hpp
  liop/liop_descriptor.hpp
  descriptor.hpp
  io_regions_type.hpp
  image_describer_akaze.hpp
  regions_factory.hpp
  feature.hpp
  features.hpp
  selection.hpp
  keypointSet.hpp
  svgVisualization.hpp
  ImageDescriberCommon.hpp
  RegionsPerView.hpp
  FeaturesPerView.hpp
)

# Sources
set(features_files_sources
  selection.cpp
  image_describer.cpp
  akaze/AKAZE.cpp
  liop/liop_descriptor.cpp
  svgVisualization.cpp
  ImageDescriberCommon.cpp
  FeaturesPerView.cpp
)

# CCTAG imageDescriber
if(ALICEVISION_HAVE_CCTAG)
  list(APPEND features_files_headers cctag/CCTAG_describer.hpp)
  list(APPEND features_files_sources cctag/CCTAG_describer.cpp)
endif()

# PopSIFT imageDescriber
if(ALICEVISION_HAVE_POPSIFT)
  list(APPEND features_files_headers sift/SIFT_popSIFT_describer.hpp)
  list(APPEND features_files_sources sift/SIFT_popSIFT_describer.cpp)
endif()

# OpenCV imageDescriber
if(ALICEVISION_HAVE_OPENCV)

  list(APPEND features_files_headers openCV/AKAZE_openCV_describer.hpp)
  list(APPEND features_files_sources openCV/AKAZE_openCV_describer.cpp)

  if(ALICEVISION_HAVE_OCVSIFT)
    list(APPEND features_files_headers openCV/SIFT_openCV_describer.hpp)
    list(APPEND features_files_sources openCV/SIFT_openCV_describer.cpp)
  endif()

endif()

set_source_files_properties(${features_files_sources} PROPERTIES LANGUAGE CXX)

add_library(aliceVision_features
  ${features_files_headers}
  ${features_files_sources}
)

target_link_libraries( aliceVision_features 
  PUBLIC aliceVision_numeric
         aliceVision_image
         aliceVision_multiview
         vlsift
         stlplus
         ${LOG_LIB}
)

# Link CCTAG library
if(ALICEVISION_HAVE_CCTAG)
  target_link_libraries(aliceVision_features PUBLIC CCTag::CCTag)
endif()

# Link PopSIFT library
if(ALICEVISION_HAVE_POPSIFT)
  target_link_libraries(aliceVision_features PUBLIC PopSift::popsift)
endif()

# Link OpenCV library
if(ALICEVISION_HAVE_OPENCV)
  target_link_libraries(aliceVision_features PUBLIC ${OpenCV_LIBS})
endif()

set_target_properties(aliceVision_features
  PROPERTIES SOVERSION ${ALICEVISION_VERSION_MAJOR}
  VERSION "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}"
)

install(TARGETS aliceVision_features
  DESTINATION lib
  EXPORT aliceVision-targets
)

set_property(TARGET aliceVision_features
  PROPERTY FOLDER AliceVision/AliceVision
)

UNIT_TEST(aliceVision features "aliceVision_features")
