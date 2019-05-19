project(feature)

set(featureData_files_headers
  imageDescriberCommon.hpp
)
set(featureData_files_sources
  imageDescriberCommon.cpp
)

# Sub-library dedicated for the data structure to cut dependencies
alicevision_add_library(aliceVision_featureData
  SOURCES ${featureData_files_headers} ${featureData_files_sources}
  PRIVATE_LINKS
    ${Boost_FILESYSTEM_LIBRARY}
)

if(ALICEVISION_BUILD_SFM)
  # Headers
  set(feature_files_headers
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
    KeypointSet.hpp
    PointFeature.hpp
    Regions.hpp
    regionsFactory.hpp
    RegionsPerView.hpp
    selection.hpp
    svgVisualization.hpp
  )

  # Sources
  set(feature_files_sources
    akaze/AKAZE.cpp
    akaze/descriptorLIOP.cpp
    akaze/ImageDescriber_AKAZE.cpp
    sift/SIFT.cpp
    FeaturesPerView.cpp
    ImageDescriber.cpp
    selection.cpp
    svgVisualization.cpp
  )

  # CCTAG ImageDescriber
  if(ALICEVISION_HAVE_CCTAG)
    list(APPEND feature_files_headers cctag/ImageDescriber_CCTAG.hpp)
    list(APPEND feature_files_sources cctag/ImageDescriber_CCTAG.cpp)
  endif()

  # PopSIFT ImageDescriber
  if(ALICEVISION_HAVE_POPSIFT)
    list(APPEND feature_files_headers sift/ImageDescriber_SIFT_popSIFT.hpp)
    list(APPEND feature_files_sources sift/ImageDescriber_SIFT_popSIFT.cpp)
  endif()

  # OpenCV ImageDescriber
  if(ALICEVISION_HAVE_OPENCV)
    list(APPEND feature_files_headers openCV/ImageDescriber_AKAZE_OCV.hpp)
    list(APPEND feature_files_sources openCV/ImageDescriber_AKAZE_OCV.cpp)

    if(ALICEVISION_HAVE_OCVSIFT)
      list(APPEND feature_files_headers openCV/ImageDescriber_SIFT_OCV.hpp)
      list(APPEND feature_files_sources openCV/ImageDescriber_SIFT_OCV.cpp)
    endif()
  endif()


  alicevision_add_library(aliceVision_feature
    SOURCES ${feature_files_headers} ${feature_files_sources}
    PUBLIC_LINKS
      aliceVision_featureData
      aliceVision_image
      aliceVision_numeric
      aliceVision_system
      aliceVision_gpu
      vlsift
    PRIVATE_LINKS
      ${Boost_FILESYSTEM_LIBRARY}
  )

  # Link CCTAG library
  if(ALICEVISION_HAVE_CCTAG)
    target_link_libraries(aliceVision_feature PUBLIC CCTag::CCTag)
  endif()

  # Link PopSIFT library
  if(ALICEVISION_HAVE_POPSIFT)
    target_link_libraries(aliceVision_feature PRIVATE PopSift::popsift)
  endif()

  # Link OpenCV library
  if(ALICEVISION_HAVE_OPENCV)
    target_link_libraries(aliceVision_feature PUBLIC ${OpenCV_LIBS})
  endif()

  # Unit tests
  alicevision_add_test(features_test.cpp NAME "feature" LINKS aliceVision_feature)

endif()
