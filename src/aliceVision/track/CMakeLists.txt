# Headers
set(tracks_files_headers
  Track.hpp
)

add_library(aliceVision_track INTERFACE)

target_link_libraries(aliceVision_track
  INTERFACE aliceVision_feature
            ${LEMON_LIBRARY}
            ${LOG_LIB}
)

UNIT_TEST(aliceVision track "aliceVision_track")

add_custom_target(aliceVision_track_ide SOURCES ${tracks_files_headers})
