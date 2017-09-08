# MOSEK library detection
find_path(MOSEK_INCLUDE NAMES mosek.h PATHS ${MOSEK_SEARCH_HEADER})
find_library(MOSEK_LIB NAMES libmosek libmosek.so libmosek64 libmosek64.so PATHS ${MOSEK_SEARCH_LIB})

if(EXISTS ${MOSEK_INCLUDE} AND EXISTS ${MOSEK_LIB})
  set(MOSEK_FOUND   true  CACHE BOOL "USE MOSEK library")
  message("-- Found Mosek header in: ${MOSEK_INCLUDE}")
  message("-- Found Mosek library: ${MOSEK_LIB}")
#  INCLUDE_DIRECTORIES( ${MOSEK_INCLUDE} )

  if(${UNIX})
    find_package(Threads REQUIRED)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CMAKE_THREAD_LIBS_INIT}")
    list(APPEND ${MOSEK_LIB} pthread)
  endif(${UNIX})

#  ADD_DEFINITIONS(-DALICEVISION_HAVE_MOSEK)

else(EXISTS ${MOSEK_INCLUDE} AND EXISTS ${MOSEK_LIB})
  message("-- Did not find MOSEK header")
  if(NOT EXISTS ${MOSEK_LIB})
    message("-- Did not find MOSEK library")
  endif()
endif(EXISTS ${MOSEK_INCLUDE} AND EXISTS ${MOSEK_LIB})

if(NOT MOSEK_FOUND)
  message(STATUS "Could not find mosek library on this machine.")
endif()

mark_as_advanced(MOSEK_FOUND)

