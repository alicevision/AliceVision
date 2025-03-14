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
    Boost::system
    Boost::boost
)

# Install DB
install(FILES cameraSensors.db
        DESTINATION ${CMAKE_INSTALL_DATADIR}/aliceVision
)

# Unit tests
alicevision_add_test(parseDatabase_test.cpp NAME "sensorDB_parseDatabase" LINKS aliceVision_sensorDB)

# SWIG Binding
if (ALICEVISION_BUILD_SWIG_BINDING)
    alicevision_swig_add_library(sensorDB
        SOURCES SensorDB.i
        PUBLIC_LINKS
            aliceVision_sensorDB
            ${Python3_LIBRARIES}
        PRIVATE_INCLUDE_DIRS
            ../include
            ${ALICEVISION_ROOT}/include
            ${Python3_INCLUDE_DIRS}
            ${Python3_NumPy_INCLUDE_DIRS}
    )
endif()
