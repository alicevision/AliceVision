cmake_minimum_required(VERSION 2.6)

set(PROJECT_NAME "LEMON")

if (COMMAND cmake_policy)
  if(POLICY CMP0048)
    CMAKE_POLICY(SET CMP0048 NEW)
  endif(POLICY CMP0048)
endif (COMMAND cmake_policy)

project(${PROJECT_NAME})

include(FindPythonInterp)
include(FindWget)

if(EXISTS ${PROJECT_SOURCE_DIR}/cmake/version.cmake)
  INCLUDE(${PROJECT_SOURCE_DIR}/cmake/version.cmake)
elseif(DEFINED ENV{LEMON_VERSION})
  set(LEMON_VERSION $ENV{LEMON_VERSION} CACHE STRING "LEMON version string.")
else()
  execute_process(
    COMMAND
    hg log -r. --template "{latesttag}"
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    OUTPUT_VARIABLE HG_REVISION_TAG
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  execute_process(
    COMMAND
    hg log -r. --template "{latesttagdistance}"
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    OUTPUT_VARIABLE HG_REVISION_DIST
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  execute_process(
    COMMAND
    hg log -r. --template "{node|short}"
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    OUTPUT_VARIABLE HG_REVISION_ID
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )

  if(HG_REVISION_TAG STREQUAL "")
    set(HG_REVISION_ID "hg-tip")
  else()
    if(HG_REVISION_TAG STREQUAL "null")
      set(HG_REVISION_TAG "trunk")
    elseif(HG_REVISION_TAG MATCHES "^r")
      string(SUBSTRING ${HG_REVISION_TAG} 1 -1 HG_REVISION_TAG)
    endif()
    if(HG_REVISION_DIST STREQUAL "0")
      set(HG_REVISION ${HG_REVISION_TAG})
    else()
      set(HG_REVISION
	"${HG_REVISION_TAG}+${HG_REVISION_DIST}-${HG_REVISION_ID}")
    endif()
  endif()

  set(LEMON_VERSION ${HG_REVISION} CACHE STRING "LEMON version string.")
endif()

set(PROJECT_VERSION ${LEMON_VERSION})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Doxygen)
find_package(Ghostscript)

set(LEMON_ENABLE_GLPK NO CACHE STRING "Enable GLPK solver backend.")
set(LEMON_ENABLE_ILOG NO CACHE STRING "Enable ILOG (CPLEX) solver backend.")
set(LEMON_ENABLE_COIN NO CACHE STRING "Enable COIN solver backend.")
set(LEMON_ENABLE_SOPLEX NO CACHE STRING "Enable SoPlex solver backend.")

if(LEMON_ENABLE_GLPK)
  find_package(GLPK 4.33)
endif(LEMON_ENABLE_GLPK)
if(LEMON_ENABLE_ILOG)
  find_package(ILOG)
endif(LEMON_ENABLE_ILOG)
if(LEMON_ENABLE_COIN)
  find_package(COIN)
endif(LEMON_ENABLE_COIN)
if(LEMON_ENABLE_SOPLEX)
  find_package(SOPLEX)
endif(LEMON_ENABLE_SOPLEX)

if(GLPK_FOUND)
  set(LEMON_HAVE_LP TRUE)
  set(LEMON_HAVE_MIP TRUE)
  set(LEMON_HAVE_GLPK TRUE)
endif(GLPK_FOUND)
if(ILOG_FOUND)
  set(LEMON_HAVE_LP TRUE)
  set(LEMON_HAVE_MIP TRUE)
  set(LEMON_HAVE_CPLEX TRUE)
endif(ILOG_FOUND)
if(COIN_FOUND)
  set(LEMON_HAVE_LP TRUE)
  set(LEMON_HAVE_MIP TRUE)
  set(LEMON_HAVE_CLP TRUE)
  set(LEMON_HAVE_CBC TRUE)
endif(COIN_FOUND)
if(SOPLEX_FOUND)
  set(LEMON_HAVE_LP TRUE)
  set(LEMON_HAVE_SOPLEX TRUE)
endif(SOPLEX_FOUND)

if(ILOG_FOUND)
  set(DEFAULT_LP "CPLEX")
  set(DEFAULT_MIP "CPLEX")
elseif(COIN_FOUND)
  set(DEFAULT_LP "CLP")
  set(DEFAULT_MIP "CBC")
elseif(GLPK_FOUND)
  set(DEFAULT_LP "GLPK")
  set(DEFAULT_MIP "GLPK")
elseif(SOPLEX_FOUND)
  set(DEFAULT_LP "SOPLEX")
endif()

if(NOT LEMON_DEFAULT_LP OR
    (NOT ILOG_FOUND AND (LEMON_DEFAULT_LP STREQUAL "CPLEX")) OR
    (NOT COIN_FOUND AND (LEMON_DEFAULT_LP STREQUAL "CLP")) OR
    (NOT GLPK_FOUND AND (LEMON_DEFAULT_LP STREQUAL "GLPK")) OR
    (NOT SOPLEX_FOUND AND (LEMON_DEFAULT_LP STREQUAL "SOPLEX")))
  set(LEMON_DEFAULT_LP ${DEFAULT_LP} CACHE STRING
    "Default LP solver backend (GLPK, CPLEX, CLP or SOPLEX)" FORCE)
endif()
if(NOT LEMON_DEFAULT_MIP OR
    (NOT ILOG_FOUND AND (LEMON_DEFAULT_MIP STREQUAL "CPLEX")) OR
    (NOT COIN_FOUND AND (LEMON_DEFAULT_MIP STREQUAL "CBC")) OR
    (NOT GLPK_FOUND AND (LEMON_DEFAULT_MIP STREQUAL "GLPK")))
  set(LEMON_DEFAULT_MIP ${DEFAULT_MIP} CACHE STRING
    "Default MIP solver backend (GLPK, CPLEX or CBC)" FORCE)
endif()


if(DEFINED ENV{LEMON_CXX_WARNING})
  set(CXX_WARNING $ENV{LEMON_CXX_WARNING})
else()
  if(CMAKE_COMPILER_IS_GNUCXX)
    set(CXX_WARNING "-Wall -W -Wunused -Wformat=2 -Wctor-dtor-privacy -Wnon-virtual-dtor -Wno-char-subscripts -Wwrite-strings -Wno-char-subscripts -Wreturn-type -Wcast-qual -Wcast-align -Wsign-promo -Woverloaded-virtual -fno-strict-aliasing -Wold-style-cast -Wno-unknown-pragmas")
    set(CMAKE_CXX_FLAGS_DEBUG CACHE STRING "-ggdb")
    set(CMAKE_C_FLAGS_DEBUG CACHE STRING "-ggdb")
  elseif(MSVC)
    # This part is unnecessary 'casue the same is set by the lemon/core.h.
    # Still keep it as an example.
    set(CXX_WARNING "/wd4250 /wd4355 /wd4503 /wd4800 /wd4996")
    # Suppressed warnings:
    # C4250: 'class1' : inherits 'class2::member' via dominance
    # C4355: 'this' : used in base member initializer list
    # C4503: 'function' : decorated name length exceeded, name was truncated
    # C4800: 'type' : forcing value to bool 'true' or 'false'
    #        (performance warning)
    # C4996: 'function': was declared deprecated
  else()
    set(CXX_WARNING "-Wall")
  endif()
endif()
set(LEMON_CXX_WARNING_FLAGS ${CXX_WARNING} CACHE STRING "LEMON warning flags.")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${LEMON_CXX_WARNING_FLAGS}")

if(MSVC)
  set( CMAKE_CXX_FLAGS_MAINTAINER "/WX ${CMAKE_CXX_FLAGS_DEBUG}" CACHE STRING
    "Flags used by the C++ compiler during maintainer builds."
    )
  set( CMAKE_C_FLAGS_MAINTAINER "/WX ${CMAKE_CXX_FLAGS_DEBUG}" CACHE STRING
    "Flags used by the C compiler during maintainer builds."
    )
  set( CMAKE_EXE_LINKER_FLAGS_MAINTAINER
    "${CMAKE_EXE_LINKER_FLAGS_DEBUG}" CACHE STRING
    "Flags used for linking binaries during maintainer builds."
    )
  set( CMAKE_SHARED_LINKER_FLAGS_MAINTAINER
    "${CMAKE_SHARED_LINKER_FLAGS_DEBUG}" CACHE STRING
    "Flags used by the shared libraries linker during maintainer builds."
    )
else()
  set( CMAKE_CXX_FLAGS_MAINTAINER "-Werror -ggdb -O0" CACHE STRING
    "Flags used by the C++ compiler during maintainer builds."
    )
  set( CMAKE_C_FLAGS_MAINTAINER "-Werror -O0" CACHE STRING
    "Flags used by the C compiler during maintainer builds."
    )
  set( CMAKE_EXE_LINKER_FLAGS_MAINTAINER
    "-Wl,--warn-unresolved-symbols,--warn-once" CACHE STRING
    "Flags used for linking binaries during maintainer builds."
    )
  set( CMAKE_SHARED_LINKER_FLAGS_MAINTAINER
    "-Wl,--warn-unresolved-symbols,--warn-once" CACHE STRING
    "Flags used by the shared libraries linker during maintainer builds."
    )
endif()

mark_as_advanced(
    CMAKE_CXX_FLAGS_MAINTAINER
    CMAKE_C_FLAGS_MAINTAINER
    CMAKE_EXE_LINKER_FLAGS_MAINTAINER
    CMAKE_SHARED_LINKER_FLAGS_MAINTAINER
)

if(CMAKE_CONFIGURATION_TYPES)
  list(APPEND CMAKE_CONFIGURATION_TYPES Maintainer)
  list(REMOVE_DUPLICATES CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_CONFIGURATION_TYPES "${CMAKE_CONFIGURATION_TYPES}" CACHE STRING
      "Add the configurations that we need"
      FORCE)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

set( CMAKE_BUILD_TYPE "${CMAKE_BUILD_TYPE}" CACHE STRING
    "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel Maintainer."
    FORCE)


include(CheckTypeSize)
CHECK_TYPE_SIZE("long long" LONG_LONG)
set(LEMON_HAVE_LONG_LONG ${HAVE_LONG_LONG})

include(FindThreads)

if(NOT LEMON_THREADING)
  if(CMAKE_USE_PTHREADS_INIT)
    set(LEMON_THREADING "Pthread")
  elseif(CMAKE_USE_WIN32_THREADS_INIT)
    set(LEMON_THREADING "Win32")
  else()
    set(LEMON_THREADING "None")
  endif()
endif()

set( LEMON_THREADING "${LEMON_THREADING}" CACHE STRING
  "Choose the threading library, options are: Pthread Win32 None."
  FORCE )

if(LEMON_THREADING STREQUAL "Pthread")
  set(LEMON_USE_PTHREAD TRUE)
elseif(LEMON_THREADING STREQUAL "Win32")
  set(LEMON_USE_WIN32_THREADS TRUE)
endif()

enable_testing()

if(${CMAKE_BUILD_TYPE} STREQUAL "Maintainer")
  add_custom_target(check ALL COMMAND ${CMAKE_CTEST_COMMAND})
else()
  add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND})
endif()

add_subdirectory(lemon)
if(${CMAKE_SOURCE_DIR} STREQUAL ${PROJECT_SOURCE_DIR})
  add_subdirectory(contrib)
  add_subdirectory(demo)
  add_subdirectory(tools)
  add_subdirectory(doc)
  add_subdirectory(test)
endif()

configure_file(
  ${PROJECT_SOURCE_DIR}/cmake/LEMONConfig.cmake.in
  ${PROJECT_BINARY_DIR}/cmake/LEMONConfig.cmake
  @ONLY
)
if(UNIX)
  install(
    FILES ${PROJECT_BINARY_DIR}/cmake/LEMONConfig.cmake
    DESTINATION share/lemon/cmake
  )
elseif(WIN32)
  install(
    FILES ${PROJECT_BINARY_DIR}/cmake/LEMONConfig.cmake
    DESTINATION cmake
  )
endif()

configure_file(
  ${PROJECT_SOURCE_DIR}/cmake/version.cmake.in
  ${PROJECT_BINARY_DIR}/cmake/version.cmake
  @ONLY
)

