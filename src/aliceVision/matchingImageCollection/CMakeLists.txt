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
  geometricFilterUtils.hpp
  pairBuilder.hpp
)

# Sources
set(matching_collection_images_files_sources
  matchingCommon.cpp
  ImageCollectionMatcher_generic.cpp
  ImageCollectionMatcher_cascadeHashing.cpp
  GeometricFilterMatrix_HGrowing.cpp
  geometricFilterUtils.cpp
  pairBuilder.cpp
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
    Boost::filesystem
  PRIVATE_LINKS
    aliceVision_system
    ${CERES_LIBRARIES}
)

# Unit tests
alicevision_add_test(pairBuilder_test.cpp           NAME "matchingImageCollection_pairBuilder"           LINKS aliceVision_matchingImageCollection)
alicevision_add_test(geometricFilterUtils_test.cpp  NAME "matchingImageCollection_geometricFilterUtils"  LINKS aliceVision_matchingImageCollection)
