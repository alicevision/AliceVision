# Headers
set(tracks_files_headers
  Track.hpp
)

# Sources
set(tracks_files_sources
  Track.cpp
)

add_library(aliceVision_track
  ${tracks_files_headers}
  ${tracks_files_sources}
)

set_property(TARGET aliceVision_track
  PROPERTY FOLDER AliceVision
)

target_link_libraries(aliceVision_track
  PUBLIC 
    aliceVision_feature
    aliceVision_matching
    aliceVision_stl
    ${LEMON_LIBRARY}
)

install(TARGETS aliceVision_track
  DESTINATION lib/
  EXPORT aliceVision-targets
)

UNIT_TEST(aliceVision track "aliceVision_track")

add_custom_target(aliceVision_track_ide SOURCES ${tracks_files_headers})

set_property(TARGET aliceVision_track_ide
  PROPERTY FOLDER AliceVision
)
