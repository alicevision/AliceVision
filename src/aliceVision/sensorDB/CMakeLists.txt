add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

# Headers
set(sensorDB_files_headers
  Datasheet.hpp
  parseDatabase.hpp
)

# Sources
set(sensorDB_files_sources
  Datasheet.cpp
  parseDatabase.cpp
)

add_library(aliceVision_sensorDB 
  ${sensorDB_files_headers}
  ${sensorDB_files_sources}
)

target_link_libraries(aliceVision_sensorDB
  ${Boost_LIBRARIES}
  ${LOG_LIB}
)

set_target_properties(aliceVision_sensorDB
  PROPERTIES SOVERSION ${ALICEVISION_VERSION_MAJOR}
  VERSION "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}"
)

# Installation rules
install(FILES sensor_width_camera_database.txt
  DESTINATION share/aliceVision
)

install(TARGETS aliceVision_sensorDB
  DESTINATION lib
  EXPORT aliceVision-targets
)

UNIT_TEST(aliceVision parseDatabase aliceVision_sensorDB)




