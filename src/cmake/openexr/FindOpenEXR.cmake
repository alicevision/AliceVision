# Module to find OpenEXR.
#
# This module will set
#   OPENEXR_FOUND          true, if found
#   OPENEXR_INCLUDE_DIR    directory where headers are found
#   OPENEXR_LIBRARIES      libraries for OpenEXR + IlmBase
#   ILMBASE_LIBRARIES      libraries just IlmBase
#   OPENEXR_VERSION        OpenEXR version (accurate for >= 2.0.0,
#                              otherwise will just guess 1.6.1)
#
# Special inputs:
#   OPENEXR_CUSTOM_INCLUDE_DIR - custom location of headers
#   OPENEXR_CUSTOM_LIB_DIR - custom location of libraries
#   OPENEXR_CUSTOM_LIB_PREFIX - special snowflake library prefix
#   OPENEXR_CUSTOM_LIB_SUFFIX - special snowflake library suffix
#

# Other standard issue macros
include (FindPackageHandleStandardArgs)
include (SelectLibraryConfigurations)

find_package (ZLIB REQUIRED)

# Link with pthreads if required
find_package (Threads)
if (CMAKE_USE_PTHREADS_INIT)
    set (ILMBASE_PTHREADS ${CMAKE_THREAD_LIBS_INIT})
endif ()

# List of likely places to find the headers -- note priority override of
# OPENEXR_CUSTOM_INCLUDE_DIR and ${OPENEXR_HOME}/include.
# ILMBASE is needed in case ilmbase an openexr are installed in separate
# directories, like NixOS does
set (GENERIC_INCLUDE_PATHS
    ${OPENEXR_CUSTOM_INCLUDE_DIR}
    ${OPENEXR_HOME}/include
    ${ILMBASE_HOME}/include
#    /usr/local/include
#    /usr/include
#    /usr/include/${CMAKE_LIBRARY_ARCHITECTURE}
#    /sw/include
#    /opt/local/include
)

# Find the include file locations. We call find_path twice -- first using
# only the custom paths, then if that fails, try the default paths only.
# This seems to be the most robust way I can find to not get confused when
# both system and custom libraries are present.
find_path (ILMBASE_INCLUDE_PATH OpenEXR/IlmBaseConfig.h
           PATHS ${GENERIC_INCLUDE_PATHS})
#find_path (ILMBASE_INCLUDE_PATH OpenEXR/IlmBaseConfig.h)
find_path (OPENEXR_INCLUDE_PATH OpenEXR/OpenEXRConfig.h
           PATHS ${GENERIC_INCLUDE_PATHS})
#find_path (OPENEXR_INCLUDE_PATH OpenEXR/OpenEXRConfig.h)

# message(WARNING "GENERIC_INCLUDE_PATHS: ${GENERIC_INCLUDE_PATHS}")
# message(WARNING "Test if file exist: ${OPENEXR_INCLUDE_PATH}/OpenEXR/ImfMultiPartInputFile.h")


# Try to figure out version number
if (EXISTS "${OPENEXR_INCLUDE_PATH}/OpenEXR/ImfMultiPartInputFile.h")
    # message(WARNING "Yes, file exist")

    # Must be at least 2.0
    file(STRINGS "${OPENEXR_INCLUDE_PATH}/OpenEXR/OpenEXRConfig.h" TMP REGEX "^#define OPENEXR_VERSION_STRING .*$")
    string (REGEX MATCHALL "[0-9]+[.0-9]+" OPENEXR_VERSION ${TMP})
    file(STRINGS "${OPENEXR_INCLUDE_PATH}/OpenEXR/OpenEXRConfig.h" TMP REGEX "^#define OPENEXR_VERSION_MAJOR .*$")
    string (REGEX MATCHALL "[0-9]+" OPENEXR_VERSION_MAJOR ${TMP})
    file(STRINGS "${OPENEXR_INCLUDE_PATH}/OpenEXR/OpenEXRConfig.h" TMP REGEX "^#define OPENEXR_VERSION_MINOR .*$")
    string (REGEX MATCHALL "[0-9]+" OPENEXR_VERSION_MINOR ${TMP})
else ()
    message(STATUS  "File ${OPENEXR_INCLUDE_PATH}/OpenEXR/ImfMultiPartInputFile.h does not exist. Default to 1.6.1")
    # Assume an old one, predates 2.x that had versions
    set (OPENEXR_VERSION 1.6.1)
    set (OPENEXR_MAJOR 1)
    set (OPENEXR_MINOR 6)
endif ()


# List of likely places to find the libraries -- note priority override of
# OPENEXR_CUSTOM_LIB_DIR and ${OPENEXR_HOME}/lib.

# If there's no OPENEXR_HOME or ILMBASE_HOME, then the path will point to
# "/lib", which may not always be wanted/expected.
if (OPENEXR_CUSTOM_LIB_DIR)
  set (GENERIC_LIBRARY_PATHS ${GENERIC_LIBRARY_PATHS} ${OPENEXR_CUSTOM_LIB_DIR})
endif()

if (OPENEXR_HOME)
  set (GENERIC_LIBRARY_PATHS ${GENERIC_LIBRARY_PATHS} ${OPENEXR_HOME})
endif()

if (ILMBASE_HOME)
  set (GENERIC_LIBRARY_PATHS ${GENERIC_LIBRARY_PATHS} ${ILMBASE_HOME})
endif()

set (GENERIC_LIBRARY_PATHS
    ${GENERIC_LIBRARY_PATHS}
    ${OPENEXR_INCLUDE_PATH}/../lib
    ${ILMBASE_INCLUDE_PATH}/../lib
    /usr/local/lib
    /usr/local/lib/${CMAKE_LIBRARY_ARCHITECTURE}
    /usr/lib
    /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}
    /sw/lib
    /opt/local/lib
    $ENV{PROGRAM_FILES}/OpenEXR/lib/static )

# Handle request for static libs by altering CMAKE_FIND_LIBRARY_SUFFIXES.
# We will restore it at the end of this file.
set (_openexr_orig_suffixes ${CMAKE_FIND_LIBRARY_SUFFIXES})
if (OpenEXR_USE_STATIC_LIBS)
    if (WIN32)
        set (CMAKE_FIND_LIBRARY_SUFFIXES .lib .a ${CMAKE_FIND_LIBRARY_SUFFIXES})
    else ()
        set (CMAKE_FIND_LIBRARY_SUFFIXES .a)
    endif ()
endif ()

# Look for the libraries themselves, for all the components. Like with the
# headers, we do two finds -- first for custom locations, then for default.
# This is complicated because the OpenEXR libraries may or may not be
# built with version numbers embedded.
set (_openexr_components IlmThread IlmImf Imath Iex Half)
foreach (COMPONENT ${_openexr_components})
    string (TOUPPER ${COMPONENT} UPPERCOMPONENT)
    # First try with the version embedded
    set (FULL_COMPONENT_NAME ${OPENEXR_CUSTOM_LIB_PREFIX}${COMPONENT}-${OPENEXR_VERSION_MAJOR}_${OPENEXR_VERSION_MINOR}${OPENEXR_CUSTOM_LIB_SUFFIX})
    find_library (OPENEXR_${UPPERCOMPONENT}_LIBRARY ${FULL_COMPONENT_NAME}
                  PATHS ${GENERIC_LIBRARY_PATHS} NO_DEFAULT_PATH)
    # Again, with no directory restrictions
    find_library (OPENEXR_${UPPERCOMPONENT}_LIBRARY ${FULL_COMPONENT_NAME})
    # Try again without the version
    set (FULL_COMPONENT_NAME ${OPENEXR_CUSTOM_LIB_PREFIX}${COMPONENT}${OPENEXR_CUSTOM_LIB_SUFFIX})
    find_library (OPENEXR_${UPPERCOMPONENT}_LIBRARY ${FULL_COMPONENT_NAME}
                  PATHS ${GENERIC_LIBRARY_PATHS} NO_DEFAULT_PATH)
    # One more time, with no restrictions
    find_library (OPENEXR_${UPPERCOMPONENT}_LIBRARY ${FULL_COMPONENT_NAME})
endforeach ()
#Half usually has no suffix
find_library (OPENEXR_HALF_LIBRARY ${OPENEXR_CUSTOM_LIB_PREFIX}Half
              PATHS ${GENERIC_LIBRARY_PATHS} NO_DEFAULT_PATH)
find_library (OPENEXR_HALF_LIBRARY ${OPENEXR_CUSTOM_LIB_PREFIX}Half)

# Set the FOUND, INCLUDE_DIR, and LIBRARIES variables.
if (ILMBASE_INCLUDE_PATH AND OPENEXR_INCLUDE_PATH AND
      OPENEXR_IMATH_LIBRARY AND OPENEXR_ILMIMF_LIBRARY AND
      OPENEXR_IEX_LIBRARY AND OPENEXR_HALF_LIBRARY)
    set (OPENEXR_FOUND TRUE)
    set (ILMBASE_FOUND TRUE)
    set (ILMBASE_INCLUDE_DIR ${ILMBASE_INCLUDE_PATH};${ILMBASE_INCLUDE_PATH}/OpenEXR CACHE STRING "The include paths needed to use IlmBase")
    set (OPENEXR_INCLUDE_DIR ${OPENEXR_INCLUDE_PATH} CACHE STRING "The include paths needed to use OpenEXR")
    set (ILMBASE_LIBRARIES ${OPENEXR_IMATH_LIBRARY} ${OPENEXR_IEX_LIBRARY} ${OPENEXR_HALF_LIBRARY} ${OPENEXR_ILMTHREAD_LIBRARY} ${ILMBASE_PTHREADS} CACHE STRING "The libraries needed to use IlmBase")
    set (OPENEXR_LIBRARIES ${OPENEXR_ILMIMF_LIBRARY} ${ILMBASE_LIBRARIES} ${ZLIB_LIBRARIES} CACHE STRING "The libraries needed to use OpenEXR")
endif ()

find_package_handle_standard_args (OpenEXR
    REQUIRED_VARS ILMBASE_INCLUDE_PATH OPENEXR_INCLUDE_PATH
                  OPENEXR_IMATH_LIBRARY OPENEXR_ILMIMF_LIBRARY
                  OPENEXR_IEX_LIBRARY OPENEXR_HALF_LIBRARY
    VERSION_VAR   OPENEXR_VERSION
    )

message(WARNING "ILMBASE_INCLUDE_DIR: ${ILMBASE_INCLUDE_DIR}")

MARK_AS_ADVANCED(
    ILMBASE_INCLUDE_DIR
    OPENEXR_INCLUDE_DIR
    ILMBASE_LIBRARIES
    OPENEXR_LIBRARIES
    OPENEXR_ILMIMF_LIBRARY
    OPENEXR_IMATH_LIBRARY
    OPENEXR_IEX_LIBRARY
    OPENEXR_HALF_LIBRARY
    OPENEXR_VERSION)

# Restore the original CMAKE_FIND_LIBRARY_SUFFIXES
set (CMAKE_FIND_LIBRARY_SUFFIXES ${_openexr_orig_suffixes})
