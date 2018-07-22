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

alicevision_add_library(aliceVision_matching
  SOURCES ${matching_files_headers} ${matching_files_sources}
  PUBLIC_LINKS
    aliceVision_feature
    aliceVision_numeric
    aliceVision_stl
    aliceVision_system
  PRIVATE_LINKS
    ${Boost_FILESYSTEM_LIBRARY}
    ${FLANN_LIBRARY}
)

# Unit tests
alicevision_add_test(matching_test.cpp NAME "matching"          LINKS aliceVision_matching)
alicevision_add_test(filters_test.cpp  NAME "matching_filters"  LINKS aliceVision_matching)
alicevision_add_test(indMatch_test.cpp NAME "matching_indMatch" LINKS aliceVision_matching)
alicevision_add_test(metric_test.cpp   NAME "matching_metric"   LINKS aliceVision_matching)

add_subdirectory(kvld)
