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
  TracksHandler.hpp
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
