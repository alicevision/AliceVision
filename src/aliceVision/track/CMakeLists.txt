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

target_link_libraries(aliceVision_track
  INTERFACE aliceVision_feature
            ${LEMON_LIBRARY}
            ${LOG_LIB}
)

install(TARGETS aliceVision_track
  DESTINATION lib/
  EXPORT aliceVision-targets
)

UNIT_TEST(aliceVision track "aliceVision_track")

add_custom_target(aliceVision_track_ide SOURCES ${tracks_files_headers})
