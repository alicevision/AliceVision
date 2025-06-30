# Headers
set(matching_files_headers
    ArrayMatcher.hpp
    ArrayMatcher_bruteForce.hpp
    ArrayMatcher_cascadeHashing.hpp
    ArrayMatcher_kdtreeFlann.hpp
    IndMatch.hpp
    MatchesCollections.hpp
    IndMatchDecorator.hpp
    filters.hpp
    guidedMatching.hpp
    io.hpp
    matcherType.hpp
    CascadeHasher.hpp
    RegionsMatcher.hpp
    pairwiseAdjacencyDisplay.hpp
    supportEstimation.hpp
    matchesFiltering.hpp
    svgVisualization.hpp
)

# Sources
set(matching_files_sources
    io.cpp
    guidedMatching.cpp
    matcherType.cpp
    RegionsMatcher.cpp
    supportEstimation.cpp
    matchesFiltering.cpp
    svgVisualization.cpp
)

alicevision_add_library(aliceVision_matching
    SOURCES ${matching_files_headers} ${matching_files_sources}
    PUBLIC_LINKS
        aliceVision_camera
        aliceVision_feature
        aliceVision_numeric
        aliceVision_stl
        aliceVision_system
    
    PRIVATE_LINKS
        Boost::boost
        ${FLANN_LIBRARIES}
)

# Unit tests
alicevision_add_test(matching_test.cpp NAME "matching"          LINKS aliceVision_matching ${FLANN_LIBRARIES})
alicevision_add_test(filters_test.cpp  NAME "matching_filters"  LINKS aliceVision_matching)
alicevision_add_test(indMatch_test.cpp NAME "matching_indMatch" LINKS aliceVision_matching)

add_subdirectory(kvld)

# SWIG Binding
if (ALICEVISION_BUILD_SWIG_BINDING)
    alicevision_swig_add_library(matching
        SOURCES matching.i
        PUBLIC_LINKS
            aliceVision_matching
            ${Python3_LIBRARIES}
        PRIVATE_INCLUDE_DIRS
            ../include
            ${ALICEVISION_ROOT}/include
            ${Python3_INCLUDE_DIRS}
            ${Python3_NumPy_INCLUDE_DIRS}
    )
endif()