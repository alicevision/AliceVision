# Headers
set(tracks_files_headers
  Track.hpp
  TracksBuilder.hpp
  tracksUtils.hpp
)

# Sources
set(tracks_files_sources
  TracksBuilder.cpp
  tracksUtils.cpp
)

alicevision_add_library(aliceVision_track
  SOURCES ${tracks_files_headers} ${tracks_files_sources}
  PUBLIC_LINKS
    aliceVision_feature
    aliceVision_matching
    aliceVision_stl
    ${LEMON_LIBRARY}
)

# Unit tests
alicevision_add_test(track_test.cpp NAME "track" LINKS aliceVision_track)
