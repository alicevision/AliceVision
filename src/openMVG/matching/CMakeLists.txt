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

add_library(openMVG_matching
  ${matching_files_headers}
  ${matching_files_sources}
)

set_target_properties(openMVG_matching
  PROPERTIES SOVERSION ${OPENMVG_VERSION_MAJOR}
  VERSION "${OPENMVG_VERSION_MAJOR}.${OPENMVG_VERSION_MINOR}"
)

target_link_libraries(openMVG_matching
  openMVG_features
  stlplus
  ${FLANN_LIBRARY}
  ${LOG_LIB}
)

set_property(TARGET openMVG_matching
  PROPERTY FOLDER OpenMVG/OpenMVG
)

install(TARGETS openMVG_matching
  DESTINATION lib
  EXPORT openMVG-targets
)

UNIT_TEST(openMVG matching          "openMVG_matching")
UNIT_TEST(openMVG matching_filters  "openMVG_matching")
UNIT_TEST(openMVG indMatch          "openMVG_matching")
UNIT_TEST(openMVG metric            "openMVG_matching")

add_subdirectory(kvld)
