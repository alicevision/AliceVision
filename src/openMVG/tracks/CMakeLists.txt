set(tracks_files_headers
  tracks.hpp
)

add_library(openMVG_tracks INTERFACE)
target_sources(openMVG_tracks INTERFACE
    ${tracks_files_headers}
    )

target_link_libraries(openMVG_tracks
  INTERFACE openMVG_features
            ${LEMON_LIBRARY}
            ${LOG_LIB}
)

UNIT_TEST(openMVG tracks "openMVG_tracks")

add_custom_target(openMVG_tracks_ide SOURCES ${tracks_files_headers})
