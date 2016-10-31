add_library(openMVG_tracks INTERFACE)
target_link_libraries(openMVG_tracks INTERFACE ${LEMON_LIBRARY} ${LOG_LIB})

UNIT_TEST(openMVG tracks "openMVG_tracks")
            
