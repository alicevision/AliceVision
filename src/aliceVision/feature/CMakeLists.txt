project(features)

# Headers
set(features_files_headers
  akaze/AKAZE.hpp
  akaze/descriptorLIOP.hpp
  akaze/descriptorMLDB.hpp
  akaze/descriptorMSURF.hpp
  akaze/ImageDescriber_AKAZE.hpp
  sift/ImageDescriber_SIFT.hpp
  sift/ImageDescriber_SIFT_vlfeat.hpp
  sift/ImageDescriber_SIFT_vlfeatFloat.hpp
  sift/SIFT.hpp
  Descriptor.hpp
  feature.hpp
  FeaturesPerView.hpp
  ImageDescriber.hpp
  imageDescriberCommon.hpp
  KeypointSet.hpp
  PointFeature.hpp
  Regions.hpp
  regionsFactory.hpp
  RegionsPerView.hpp
  selection.hpp
  svgVisualization.hpp
)

# Sources
set(features_files_sources
  akaze/AKAZE.cpp
  akaze/descriptorLIOP.cpp
  akaze/ImageDescriber_AKAZE.cpp
  sift/SIFT.cpp
  FeaturesPerView.cpp
  ImageDescriber.cpp
  imageDescriberCommon.cpp
  selection.cpp
  svgVisualization.cpp
)

# CCTAG ImageDescriber
if(ALICEVISION_HAVE_CCTAG)
  list(APPEND features_files_headers cctag/ImageDescriber_CCTAG.hpp)
  list(APPEND features_files_sources cctag/ImageDescriber_CCTAG.cpp)
endif()

# PopSIFT ImageDescriber
if(ALICEVISION_HAVE_POPSIFT)
  list(APPEND features_files_headers sift/ImageDescriber_SIFT_popSIFT.hpp)
  list(APPEND features_files_sources sift/ImageDescriber_SIFT_popSIFT.cpp)
endif()

# OpenCV ImageDescriber
if(ALICEVISION_HAVE_OPENCV)

  list(APPEND features_files_headers openCV/ImageDescriber_AKAZE_OCV.hpp)
  list(APPEND features_files_sources openCV/ImageDescriber_AKAZE_OCV.cpp)

  if(ALICEVISION_HAVE_OCVSIFT)
    list(APPEND features_files_headers openCV/ImageDescriber_SIFT_OCV.hpp)
    list(APPEND features_files_sources openCV/ImageDescriber_SIFT_OCV.cpp)
  endif()

endif()

set_source_files_properties(${features_files_sources} PROPERTIES LANGUAGE CXX)

add_library(aliceVision_feature
  ${features_files_headers}
  ${features_files_sources}
)

target_link_libraries(aliceVision_feature
  PUBLIC
    aliceVision_image
    aliceVision_numeric
    aliceVision_system
    vlsift
  PRIVATE
    ${Boost_FILESYSTEM_LIBRARY}
)

# Link CCTAG library
if(ALICEVISION_HAVE_CCTAG)
  target_link_libraries(aliceVision_feature PUBLIC CCTag::CCTag)
endif()

# Link PopSIFT library
if(ALICEVISION_HAVE_POPSIFT)
  target_link_libraries(aliceVision_feature PUBLIC PopSift::popsift)
endif()

# Link OpenCV library
if(ALICEVISION_HAVE_OPENCV)
  target_link_libraries(aliceVision_feature PUBLIC ${OpenCV_LIBS})
endif()

set_target_properties(aliceVision_feature
  PROPERTIES SOVERSION ${ALICEVISION_VERSION_MAJOR}
  VERSION "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}"
)

install(TARGETS aliceVision_feature
  DESTINATION lib
  EXPORT aliceVision-targets
)

set_property(TARGET aliceVision_feature
  PROPERTY FOLDER AliceVision
)

UNIT_TEST(aliceVision features "aliceVision_feature")
