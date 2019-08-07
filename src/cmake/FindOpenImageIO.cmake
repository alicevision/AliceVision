find_package (OpenEXR REQUIRED)

macro(OPENIMAGEIO_RESET_FIND_LIBRARY_PREFIX)
  if (MSVC)
    set(CMAKE_FIND_LIBRARY_PREFIXES "${CALLERS_CMAKE_FIND_LIBRARY_PREFIXES}")
  endif (MSVC)
endmacro(OPENIMAGEIO_RESET_FIND_LIBRARY_PREFIX)

# Called if we failed to find OpenImageIO or any of it's required dependencies,
# unsets all public (designed to be used externally) variables and reports
# error message at priority depending upon [REQUIRED/QUIET/<NONE>] argument.
macro(OPENIMAGEIO_REPORT_NOT_FOUND REASON_MSG)
  unset(OPENIMAGEIO_FOUND)
  unset(OPENIMAGEIO_INCLUDE_DIRS)
  unset(OPENIMAGEIO_LIBRARIES)
  # Make results of search visible in the CMake GUI if OpenImageIO has not
  # been found so that user does not have to toggle to advanced view.
  mark_as_advanced(CLEAR OPENIMAGEIO_INCLUDE_DIR
                         OPENIMAGEIO_LIBRARY)

  openimageio_reset_find_library_prefix()

  # Note <package>_FIND_[REQUIRED/QUIETLY] variables defined by FindPackage()
  # use the camelcase library name, not uppercase.
  if (OpenImageIO_FIND_QUIETLY)
    message(STATUS "Failed to find OpenImageIO - " ${REASON_MSG} ${ARGN})
  elseif (OpenImageIO_FIND_REQUIRED)
    message(FATAL_ERROR "Failed to find OpenImageIO - " ${REASON_MSG} ${ARGN})
  else()
    # Neither QUIETLY nor REQUIRED, use no priority which emits a message
    # but continues configuration and allows generation.
    message("-- Failed to find OpenImageIO - " ${REASON_MSG} ${ARGN})
  endif ()
endmacro(OPENIMAGEIO_REPORT_NOT_FOUND)

# Handle possible presence of lib prefix for libraries on MSVC, see
# also OPENIMAGEIO_RESET_FIND_LIBRARY_PREFIX().
if (MSVC)
  # Preserve the caller's original values for CMAKE_FIND_LIBRARY_PREFIXES
  # s/t we can set it back before returning.
  set(CALLERS_CMAKE_FIND_LIBRARY_PREFIXES "${CMAKE_FIND_LIBRARY_PREFIXES}")
  # The empty string in this list is important, it represents the case when
  # the libraries have no prefix (shared libraries / DLLs).
  set(CMAKE_FIND_LIBRARY_PREFIXES "lib" "" "${CMAKE_FIND_LIBRARY_PREFIXES}")
endif (MSVC)

# Search user-installed locations first, so that we prefer user installs
# to system installs where both exist.
list(APPEND OPENIMAGEIO_CHECK_INCLUDE_DIRS
  /usr/local/include
  /usr/local/homebrew/include # Mac OS X
  /opt/local/var/macports/software # Mac OS X.
  /opt/local/include
  /usr/include)
# Windows (for C:/Program Files prefix).
list(APPEND OPENIMAGEIO_CHECK_PATH_SUFFIXES
  include
  Include
  openimageio/include
  openimageio/Include
  OpenImageIO/include
  OpenImageIO/Include)

list(APPEND OPENIMAGEIO_CHECK_LIBRARY_DIRS
  /usr/local/lib
  /usr/local/homebrew/lib # Mac OS X.
  /opt/local/lib
  /usr/lib)
# Windows (for C:/Program Files prefix).
list(APPEND OPENIMAGEIO_CHECK_LIBRARY_SUFFIXES
  lib
  Lib
  lib64
  Lib64
  openimageio/lib
  openimageio/Lib
  OpenImageIO/lib
  OpenImageIO/Lib)


# Search supplied hint directories first if supplied.
find_path(OPENIMAGEIO_INCLUDE_DIR
  NAMES OpenImageIO/imageio.h
  HINTS
        ${OPENIMAGEIO_INCLUDE_DIR_HINTS}
        #${OPENIMAGEIO_CHECK_INCLUDE_DIRS}
  PATH_SUFFIXES ${OPENIMAGEIO_CHECK_PATH_SUFFIXES})
if (NOT OPENIMAGEIO_INCLUDE_DIR OR
    NOT EXISTS ${OPENIMAGEIO_INCLUDE_DIR})
  openimageio_report_not_found(
    "Could not find OpenImageIO include directory, set OPENIMAGEIO_INCLUDE_DIR "
    "to directory containing OpenImageIO/imageio.h")
endif (NOT OPENIMAGEIO_INCLUDE_DIR OR
       NOT EXISTS ${OPENIMAGEIO_INCLUDE_DIR})

find_library(OPENIMAGEIO_LIBRARY NAMES OpenImageIO
  HINTS
        ${OPENIMAGEIO_LIBRARY_DIR_HINTS}
        # ${OPENIMAGEIO_CHECK_LIBRARY_DIRS}
  PATH_SUFFIXES ${OPENIMAGEIO_CHECK_LIBRARY_SUFFIXES})
if (NOT OPENIMAGEIO_LIBRARY OR
    NOT EXISTS ${OPENIMAGEIO_LIBRARY})
  openimageio_report_not_found(
    "Could not find OpenImageIO library, set OPENIMAGEIO_LIBRARY "
    "to full path to libopenimageio.")
endif (NOT OPENIMAGEIO_LIBRARY OR
       NOT EXISTS ${OPENIMAGEIO_LIBRARY})

# Mark internally as found, then verify. OPENIMAGEIO_REPORT_NOT_FOUND() unsets
# if called.
set(OPENIMAGEIO_FOUND TRUE)

# OpenImageIO does not seem to provide any record of the version in its
# source tree, thus cannot extract version.

# Catch case when caller has set OPENIMAGEIO_INCLUDE_DIR in the cache / GUI and
# thus FIND_[PATH/LIBRARY] are not called, but specified locations are
# invalid, otherwise we would report the library as found.
if (OPENIMAGEIO_INCLUDE_DIR AND
    NOT EXISTS ${OPENIMAGEIO_INCLUDE_DIR}/OpenImageIO/imageio.h)
  openimageio_report_not_found(
    "Caller defined OPENIMAGEIO_INCLUDE_DIR:"
    " ${OPENIMAGEIO_INCLUDE_DIR} does not contain OpenImageIO/imageio.h header.")
endif (OPENIMAGEIO_INCLUDE_DIR AND
       NOT EXISTS ${OPENIMAGEIO_INCLUDE_DIR}/OpenImageIO/imageio.h)
string(TOLOWER "${OPENIMAGEIO_LIBRARY}" LOWERCASE_OPENIMAGEIO_LIBRARY)
if (OPENIMAGEIO_LIBRARY AND
    NOT "${LOWERCASE_OPENIMAGEIO_LIBRARY}" MATCHES ".*openimageio[^/]*")
  openimageio_report_not_found(
    "Caller defined OPENIMAGEIO_LIBRARY: "
    "${OPENIMAGEIO_LIBRARY} does not match OpenImageIO.")
endif (OPENIMAGEIO_LIBRARY AND
       NOT "${LOWERCASE_OPENIMAGEIO_LIBRARY}" MATCHES ".*openimageio[^/]*")

# Set standard CMake FindPackage variables if found.
if (OPENIMAGEIO_FOUND)
  set(OPENIMAGEIO_INCLUDE_DIRS ${OPENIMAGEIO_INCLUDE_DIR} ${OPENEXR_INCLUDE_DIR} ${ILMBASE_INCLUDE_DIR})
  set(OPENIMAGEIO_LIBRARIES ${OPENIMAGEIO_LIBRARY} ${ILMBASE_LIBRARIES})
endif (OPENIMAGEIO_FOUND)

openimageio_reset_find_library_prefix()

# Handle REQUIRED / QUIET optional arguments.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenImageIO DEFAULT_MSG
  OPENIMAGEIO_INCLUDE_DIRS OPENIMAGEIO_LIBRARIES)

# Only mark internal variables as advanced if we found OpenImageIO, otherwise
# leave them visible in the standard GUI for the user to set manually.
if (OPENIMAGEIO_FOUND)
  mark_as_advanced(FORCE OPENIMAGEIO_INCLUDE_DIR
                         OPENIMAGEIO_LIBRARY)
endif (OPENIMAGEIO_FOUND)
