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
if(OPENMVG_HAVE_CCTAG)
  list(APPEND features_files_headers cctag/CCTAG_describer.hpp)
  list(APPEND features_files_sources cctag/CCTAG_describer.cpp)
endif()

# PopSIFT imageDescriber
if(OPENMVG_HAVE_POPSIFT)
  list(APPEND features_files_headers sift/SIFT_popSIFT_describer.hpp)
  list(APPEND features_files_sources sift/SIFT_popSIFT_describer.cpp)
endif()

# OpenCV imageDescriber
if(OPENMVG_HAVE_OPENCV)

  list(APPEND features_files_headers openCV/AKAZE_openCV_describer.hpp)
  list(APPEND features_files_sources openCV/AKAZE_openCV_describer.cpp)

  if(OPENMVG_HAVE_OCVSIFT)
    list(APPEND features_files_headers openCV/SIFT_openCV_describer.hpp)
    list(APPEND features_files_sources openCV/SIFT_openCV_describer.cpp)
  endif()

endif()

set_source_files_properties(${features_files_sources} PROPERTIES LANGUAGE CXX)

add_library(openMVG_features
  ${features_files_headers}
  ${features_files_sources}
)

target_link_libraries( openMVG_features 
  PUBLIC openMVG_numeric
         openMVG_image
         openMVG_multiview
         vlsift
         stlplus
         ${LOG_LIB}
)

# Link CCTAG library
if(OPENMVG_HAVE_CCTAG)
  target_link_libraries(openMVG_features PUBLIC CCTag::CCTag)
endif()

# Link PopSIFT library
if(OPENMVG_HAVE_POPSIFT)
  target_link_libraries(openMVG_features PUBLIC PopSift::popsift)
endif()

# Link OpenCV library
if(OPENMVG_HAVE_OPENCV)
  target_link_libraries(openMVG_features PUBLIC ${OpenCV_LIBS})
endif()

set_target_properties(openMVG_features
  PROPERTIES SOVERSION ${OPENMVG_VERSION_MAJOR}
  VERSION "${OPENMVG_VERSION_MAJOR}.${OPENMVG_VERSION_MINOR}"
)

install(TARGETS openMVG_features
  DESTINATION lib
  EXPORT openMVG-targets
)

set_property(TARGET openMVG_features
  PROPERTY FOLDER OpenMVG/OpenMVG
)

UNIT_TEST(openMVG features "openMVG_features")
