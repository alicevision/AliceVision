# Headers
set(matching_collection_images_files_headers
  Matcher.hpp
  MatchingCommon.hpp
  Matcher_Regions_AllInMemory.hpp
  Cascade_Hashing_Matcher_Regions_AllInMemory.hpp
  E_ACRobust.hpp
  F_ACRobust.hpp
  H_ACRobust.hpp
  GeometricFilter.hpp
  GeometricFilterMatrix.hpp
  Geometric_Filter_utils.hpp
  Pair_Builder.hpp
)

# Sources
set(matching_collection_images_files_sources
  Matcher.hpp
  MatchingCommon.cpp
  Matcher_Regions_AllInMemory.cpp
  Cascade_Hashing_Matcher_Regions_AllInMemory.cpp
)

add_library(aliceVision_matching_image_collection
  ${matching_collection_images_files_headers}
  ${matching_collection_images_files_sources}
)

target_link_libraries(aliceVision_matching_image_collection
  aliceVision_matching
  ${LOG_LIB}
)

set_target_properties(aliceVision_matching_image_collection
  PROPERTIES SOVERSION ${ALICEVISION_VERSION_MAJOR}
  VERSION "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}"
)

set_property(TARGET aliceVision_matching_image_collection
  PROPERTY FOLDER AliceVision
)

install(TARGETS aliceVision_matching_image_collection
  DESTINATION lib
  EXPORT aliceVision-targets
)

UNIT_TEST(aliceVision Pair_Builder "aliceVision_matching_image_collection")
