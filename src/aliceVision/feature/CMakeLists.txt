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
    sift/ImageDescriber_DSPSIFT_vlfeat.hpp
    sift/SIFT.hpp
    virtual/ImageDescriber_Roma.hpp
    Descriptor.hpp
    feature.hpp
    FeaturesPerView.hpp
    Hamming.hpp
    ImageDescriber.hpp
    imageDescriberCommon.hpp
    imageStats.hpp
    KeypointSet.hpp
    metric.hpp
    PointFeature.hpp
    Regions.hpp
    regionsFactory.hpp
    RegionsPerView.hpp
)

# Sources
set(features_files_sources
    akaze/AKAZE.cpp
    akaze/descriptorLIOP.cpp
    akaze/ImageDescriber_AKAZE.cpp
    sift/SIFT.cpp
    sift/ImageDescriber_DSPSIFT_vlfeat.cpp
    virtual/ImageDescriber_Roma.cpp
    FeaturesPerView.cpp
    ImageDescriber.cpp
    imageDescriberCommon.cpp
    imageStats.cpp
)

# CCTAG ImageDescriber
if (ALICEVISION_HAVE_CCTAG)
    list(APPEND features_files_headers cctag/ImageDescriber_CCTAG.hpp)
    list(APPEND features_files_sources cctag/ImageDescriber_CCTAG.cpp)
endif()

# APRILTAG ImageDescriber
if (ALICEVISION_HAVE_APRILTAG)
    list(APPEND features_files_headers apriltag/ImageDescriber_APRILTAG.hpp)
    list(APPEND features_files_sources apriltag/ImageDescriber_APRILTAG.cpp)
endif()

# PopSIFT ImageDescriber
if (ALICEVISION_HAVE_POPSIFT)
    list(APPEND features_files_headers sift/ImageDescriber_SIFT_popSIFT.hpp)
    list(APPEND features_files_sources sift/ImageDescriber_SIFT_popSIFT.cpp)
endif()

# OpenCV ImageDescriber
if (ALICEVISION_HAVE_OPENCV)
    list(APPEND features_files_headers openCV/ImageDescriber_AKAZE_OCV.hpp)
    list(APPEND features_files_sources openCV/ImageDescriber_AKAZE_OCV.cpp)

    if (ALICEVISION_HAVE_OCVSIFT)
        list(APPEND features_files_headers openCV/ImageDescriber_SIFT_OCV.hpp)
        list(APPEND features_files_sources openCV/ImageDescriber_SIFT_OCV.cpp)
    endif()
endif()


alicevision_add_library(aliceVision_feature
    SOURCES ${features_files_headers} ${features_files_sources}
    PUBLIC_LINKS
        aliceVision_image
        aliceVision_numeric
        aliceVision_system
        aliceVision_gpu
        vlsift
    PRIVATE_LINKS
        Boost::boost
)

# Link CCTAG library
if (ALICEVISION_HAVE_CCTAG)
    target_link_libraries(aliceVision_feature PUBLIC CCTag::CCTag)
endif()

# Link APRILTAG library
if (ALICEVISION_HAVE_APRILTAG)
    target_link_libraries(aliceVision_feature PUBLIC apriltag::apriltag)
endif()

# Link PopSIFT library
if (ALICEVISION_HAVE_POPSIFT)
    target_link_libraries(aliceVision_feature PRIVATE PopSift::popsift)
endif()

# Link OpenCV library
if (ALICEVISION_HAVE_OPENCV)
    target_link_libraries(aliceVision_feature PUBLIC ${OpenCV_LIBS})
endif()

# Unit tests
alicevision_add_test(features_test.cpp NAME "features" LINKS aliceVision_feature)
alicevision_add_test(metric_test.cpp   NAME "descriptor_metric"   LINKS aliceVision_feature)

# SWIG Binding
if (ALICEVISION_BUILD_SWIG_BINDING)
    alicevision_swig_add_library(feature
        SOURCES feature.i
        PUBLIC_LINKS
            aliceVision_feature
            ${Python3_LIBRARIES}
        PRIVATE_INCLUDE_DIRS
            ../include
            ${ALICEVISION_ROOT}/include
            ${Python3_INCLUDE_DIRS}
            ${Python3_NumPy_INCLUDE_DIRS}
    )
endif()