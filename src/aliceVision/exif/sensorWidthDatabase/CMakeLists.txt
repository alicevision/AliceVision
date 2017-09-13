add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

UNIT_TEST(aliceVision parseDatabase "stlplus")

# Installation rules
install(FILES sensor_width_camera_database.txt
  DESTINATION share/aliceVision
)
