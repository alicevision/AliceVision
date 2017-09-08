# Headers
set(tracks_files_headers
  tracks.hpp
)

add_library(aliceVision_tracks INTERFACE)

target_link_libraries(aliceVision_tracks
  INTERFACE aliceVision_features
            ${LEMON_LIBRARY}
            ${LOG_LIB}
)

UNIT_TEST(aliceVision tracks "aliceVision_tracks")

add_custom_target(aliceVision_tracks_ide SOURCES ${tracks_files_headers})
