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

add_library(aliceVision_matchingImageCollection
  ${matching_collection_images_files_headers}
  ${matching_collection_images_files_sources}
)

target_link_libraries(aliceVision_matchingImageCollection
PUBLIC
  aliceVision_feature
  aliceVision_matching
  aliceVision_multiview
  aliceVision_robustEstimation
  aliceVision_sfm
  ${Boost_LIBRARIES}
PRIVATE
  aliceVision_system
  ${CERES_LIBRARIES}
)

set_target_properties(aliceVision_matchingImageCollection
  PROPERTIES SOVERSION ${ALICEVISION_VERSION_MAJOR}
  VERSION "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}"
)

set_property(TARGET aliceVision_matchingImageCollection
  PROPERTY FOLDER AliceVision
)

install(TARGETS aliceVision_matchingImageCollection
  DESTINATION lib
  EXPORT aliceVision-targets
)

UNIT_TEST(aliceVision pairBuilder "aliceVision_matchingImageCollection")
UNIT_TEST(aliceVision geometricFilterUtils "aliceVision_matchingImageCollection")
