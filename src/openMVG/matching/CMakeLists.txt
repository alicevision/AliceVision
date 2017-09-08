# Headers
set(matching_files_headers
  indMatch.hpp
  indMatch_utils.hpp
  indMatchDecoratorXY.hpp
  matcher_type.hpp
  matcher_brute_force.hpp
  matcher_kdtree_flann.hpp
  matcher_cascade_hashing.hpp
  matching_filters.hpp
  matching_interface.hpp
  metric.hpp
  metric_hamming.hpp
  regions_matcher.hpp
  cascade_hasher.hpp
  pairwiseAdjacencyDisplay.hpp
)

# Sources
set(matching_files_sources
  indMatch_utils.cpp
  matcher_type.cpp
  regions_matcher.cpp
)

set_source_files_properties(${matching_files_sources} PROPERTIES LANGUAGE CXX)

add_library(aliceVision_matching
  ${matching_files_headers}
  ${matching_files_sources}
)

set_target_properties(aliceVision_matching
  PROPERTIES SOVERSION ${ALICEVISION_VERSION_MAJOR}
  VERSION "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}"
)

target_link_libraries(aliceVision_matching
  aliceVision_features
  stlplus
  ${FLANN_LIBRARY}
  ${LOG_LIB}
)

set_property(TARGET aliceVision_matching
  PROPERTY FOLDER AliceVision/AliceVision
)

install(TARGETS aliceVision_matching
  DESTINATION lib
  EXPORT aliceVision-targets
)

UNIT_TEST(aliceVision matching          "aliceVision_matching")
UNIT_TEST(aliceVision matching_filters  "aliceVision_matching")
UNIT_TEST(aliceVision indMatch          "aliceVision_matching")
UNIT_TEST(aliceVision metric            "aliceVision_matching")

add_subdirectory(kvld)
