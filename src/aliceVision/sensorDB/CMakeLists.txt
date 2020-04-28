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

alicevision_add_library(aliceVision_sensorDB
  SOURCES ${sensorDB_files_headers} ${sensorDB_files_sources}
  PRIVATE_LINKS
    Boost::filesystem
    Boost::system
    Boost::boost
)

# Install DB
install(FILES cameraSensors.db
        DESTINATION ${CMAKE_INSTALL_DATADIR}/aliceVision
)

# Unit tests
alicevision_add_test(parseDatabase_test.cpp NAME "sensorDB_parseDatabase" LINKS aliceVision_sensorDB Boost::filesystem)



