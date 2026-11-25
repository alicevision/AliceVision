# Camera sensor database (cameraSensors.db) used by CameraInit to convert focal lengths into pixels.
# It is maintained in its own repository: https://github.com/alicevision/sensordb
#
# Included from the top-level CMakeLists.txt when ALICEVISION_INSTALL_SENSORDB is ON:
# defines the "sensordb" target (part of ALL) that downloads the database into the build
# directory, and installs it into share/aliceVision, where Meshroom looks for it.
#
# Also run in script mode by that target (cmake -P) to do the download itself:
#   -DSENSORDB_URL=<url> -DSENSORDB_OUTPUT=<file>

if(CMAKE_SCRIPT_MODE_FILE)
    # ── Script mode: fetch the latest database on every build ──
    # If the download fails (e.g. offline) and a previous copy exists, keep that copy.
    message(STATUS "Downloading the sensor database: ${SENSORDB_URL}")
    # Download to a temporary name: an interrupted or corrupted download never leaves a file
    # that looks valid at the final path.
    set(_partial "${SENSORDB_OUTPUT}.part")
    file(DOWNLOAD "${SENSORDB_URL}" "${_partial}"
         TLS_VERIFY ON
         STATUS _status)
    list(GET _status 0 _statusCode)
    set(_error "")
    if(NOT _statusCode EQUAL 0)
        list(GET _status 1 _statusMessage)
        set(_error "download failed: ${_statusMessage}")
    else()
        # Sanity check: lines are "make;model;sensorWidth;source".
        file(STRINGS "${_partial}" _firstLine LIMIT_COUNT 1)
        if(NOT _firstLine MATCHES "^[^;]+;[^;]+;[0-9.]+")
            set(_error "downloaded file does not look like a sensor database")
        endif()
    endif()

    if(_error)
        file(REMOVE "${_partial}")
        if(EXISTS "${SENSORDB_OUTPUT}")
            message(WARNING "Sensor database from ${SENSORDB_URL}: ${_error}. "
                            "Keeping the existing copy: ${SENSORDB_OUTPUT}")
            return()
        endif()
        message(FATAL_ERROR "Sensor database from ${SENSORDB_URL}: ${_error}")
    endif()

    # Only replace the file when its content changed, so an unchanged database is not reinstalled.
    if(EXISTS "${SENSORDB_OUTPUT}")
        file(SHA256 "${SENSORDB_OUTPUT}" _existingHash)
        file(SHA256 "${_partial}" _downloadedHash)
        if(_existingHash STREQUAL _downloadedHash)
            file(REMOVE "${_partial}")
            return()
        endif()
    endif()
    file(RENAME "${_partial}" "${SENSORDB_OUTPUT}")
    return()
endif()

# ── Configure mode ──

# Latest version of the database, from the default branch of https://github.com/alicevision/sensordb
set(ALICEVISION_SENSORDB_URL
    "https://raw.githubusercontent.com/alicevision/sensordb/develop/cameraSensors.db"
    CACHE STRING "URL of the camera sensor database (cameraSensors.db) to download")

set(_sensorDbFile "${CMAKE_BINARY_DIR}/sensordb/cameraSensors.db")

# Download at build time (not configure time), so that reconfiguring never downloads.
add_custom_target(sensordb ALL
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/sensordb"
    COMMAND ${CMAKE_COMMAND}
            -DSENSORDB_URL=${ALICEVISION_SENSORDB_URL}
            -DSENSORDB_OUTPUT=${_sensorDbFile}
            -P "${CMAKE_CURRENT_LIST_FILE}"
    COMMENT "Updating the camera sensor database"
    VERBATIM
)

install(FILES "${_sensorDbFile}"
    DESTINATION ${CMAKE_INSTALL_DATADIR}/aliceVision
)
