# Headers
set(matching_files_headers
  ArrayMatcher.hpp
  ArrayMatcher_bruteForce.hpp
  ArrayMatcher_cascadeHashing.hpp
  ArrayMatcher_kdtreeFlann.hpp
  IndMatch.hpp
  IndMatchDecorator.hpp
  filters.hpp
  io.hpp
  matcherType.hpp
  metric.hpp
  Hamming.hpp
  CascadeHasher.hpp
  RegionsMatcher.hpp
  pairwiseAdjacencyDisplay.hpp
)

# Sources
set(matching_files_sources
  io.cpp
  matcherType.cpp
  RegionsMatcher.cpp
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
  PUBLIC
    aliceVision_feature
    aliceVision_numeric
    aliceVision_stl
    aliceVision_system
  PRIVATE
    ${Boost_FILESYSTEM_LIBRARY}
    ${FLANN_LIBRARY}
)

set_property(TARGET aliceVision_matching
  PROPERTY FOLDER AliceVision
)

install(TARGETS aliceVision_matching
  DESTINATION lib
  EXPORT aliceVision-targets
)

UNIT_TEST(aliceVision matching  "aliceVision_matching")
UNIT_TEST(aliceVision filters   "aliceVision_matching")
UNIT_TEST(aliceVision indMatch  "aliceVision_matching")
UNIT_TEST(aliceVision metric    "aliceVision_matching")

add_subdirectory(kvld)
