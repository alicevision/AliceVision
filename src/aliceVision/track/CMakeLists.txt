# Headers
set(tracks_files_headers
    Track.hpp
    TracksBuilder.hpp
    TracksHandler.hpp
    tracksUtils.hpp
    trackIO.hpp
)

# Sources
set(tracks_files_sources
    TracksBuilder.cpp
    TracksHandler.cpp
    tracksUtils.cpp
    trackIO.cpp
)

alicevision_add_library(aliceVision_track
    SOURCES ${tracks_files_headers} ${tracks_files_sources}
    PUBLIC_LINKS
        aliceVision_feature
        aliceVision_matching
        aliceVision_stl
        Boost::json
    PRIVATE_LINKS
        ${LEMON_LIBRARY}
)

# Unit tests
alicevision_add_test(track_test.cpp NAME "track" LINKS aliceVision_track)

# SWIG Binding
if (ALICEVISION_BUILD_SWIG_BINDING)
    alicevision_swig_add_library(track
        SOURCES track.i
        PUBLIC_LINKS
            aliceVision_track
            ${Python3_LIBRARIES}
        PRIVATE_INCLUDE_DIRS
            ../include
            ${ALICEVISION_ROOT}/include
            ${Python3_INCLUDE_DIRS}
            ${Python3_NumPy_INCLUDE_DIRS}
    )
endif()