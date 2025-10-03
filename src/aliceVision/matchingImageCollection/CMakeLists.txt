# Headers
set(matching_collection_images_files_headers
    matchingCommon.hpp
    IImageCollectionMatcher.hpp
    ImageCollectionMatcher_generic.hpp
    ImageCollectionMatcher_cascadeHashing.hpp
    GeometricFilter.hpp
    GeometricFilterMatrix.hpp
    GeometricFilterMatrix_E_AC.hpp
    GeometricFilterMatrix_F_AC.hpp
    GeometricFilterMatrix_H_AC.hpp
    GeometricFilterMatrix_HGrowing.hpp
    GeometricFilterType.hpp
    ImagePairListIO.hpp
    geometricFilterUtils.hpp
    pairBuilder.hpp
    GeometricInfo.hpp
)

# Sources
set(matching_collection_images_files_sources
    matchingCommon.cpp
    ImageCollectionMatcher_generic.cpp
    ImageCollectionMatcher_cascadeHashing.cpp
    GeometricFilter.cpp
    GeometricFilterMatrix_HGrowing.cpp
    geometricFilterUtils.cpp
    ImagePairListIO.cpp
    pairBuilder.cpp
    GeometricInfo.cpp
)

alicevision_add_library(aliceVision_matchingImageCollection
    SOURCES ${matching_collection_images_files_headers} ${matching_collection_images_files_sources}
    PUBLIC_LINKS
        aliceVision_feature
        aliceVision_matching
        aliceVision_multiview
        aliceVision_robustEstimation
        aliceVision_sfmData
        Boost::boost
        Boost::timer
        Boost::json
    PRIVATE_LINKS
        aliceVision_system
        ${CERES_LIBRARIES}
        ${FLANN_LIBRARIES}
)

# Unit tests
alicevision_add_test(ImagePairListIO_test.cpp
    NAME "matchingImageCollection_ImagePairListIO"
    LINKS aliceVision_matchingImageCollection)

alicevision_add_test(pairBuilder_test.cpp
    NAME "matchingImageCollection_pairBuilder"
    LINKS aliceVision_matchingImageCollection
)
alicevision_add_test(geometricFilterUtils_test.cpp
    NAME "matchingImageCollection_geometricFilterUtils"
    LINKS aliceVision_matchingImageCollection
)

# SWIG Binding
if (ALICEVISION_BUILD_SWIG_BINDING)
    alicevision_swig_add_library(matchingImageCollection
        SOURCES matchingImageCollection.i
        PUBLIC_LINKS
            aliceVision_matchingImageCollection
            ${Python3_LIBRARIES}
        PRIVATE_INCLUDE_DIRS
            ../include
            ${ALICEVISION_ROOT}/include
            ${Python3_INCLUDE_DIRS}
            ${Python3_NumPy_INCLUDE_DIRS}
    )
endif()