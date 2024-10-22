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
    set(UseSWIG_TARGET_NAME_PREFERENCE STANDARD)
    set_property(SOURCE SensorDB.i PROPERTY CPLUSPLUS ON)
    set_property(SOURCE SensorDB.i PROPERTY SWIG_MODULE_NAME sensorDB)

    swig_add_library(sensorDB
        TYPE MODULE
        LANGUAGE python
        SOURCES SensorDB.i
    )

    if (CMAKE_SYSTEM_NAME MATCHES "Linux")
        list(APPEND SWIG_EXTRA_COMPILE_OPTIONS "-DSWIGWORDSIZE64")
    endif()

    set_property(
        TARGET sensorDB
        PROPERTY SWIG_COMPILE_OPTIONS -doxygen ${SWIG_EXTRA_COMPILE_OPTIONS}
    )

    target_include_directories(sensorDB
    PRIVATE
        ../include
        ${ALICEVISION_ROOT}/include
        ${Python3_INCLUDE_DIRS}
        ${Python3_NumPy_INCLUDE_DIRS}
    )
    set_property(
        TARGET sensorDB
        PROPERTY SWIG_USE_TARGET_INCLUDE_DIRECTORIES ON
    )
    set_property(
        TARGET sensorDB
        PROPERTY COMPILE_OPTIONS -std=c++17
    )

    target_link_libraries(sensorDB
    PUBLIC
        aliceVision_sensorDB
        ${Python3_LIBRARIES}
    )

    install(
    TARGETS
        sensorDB
    DESTINATION
        ${CMAKE_INSTALL_PREFIX}
    )
    install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/sensorDB.py
    DESTINATION
        ${CMAKE_INSTALL_PREFIX}
    )
endif()