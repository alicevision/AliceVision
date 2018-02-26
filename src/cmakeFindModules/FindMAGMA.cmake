# - Find the MAGMA library
#
# Usage:
#   find_package(MAGMA [REQUIRED] [QUIET] )
#
# It sets the following variables:
#   MAGMA_FOUND               ... true if magma is found on the system
#   MAGMA_LIBRARY_DIRS        ... full path to magma library
#   MAGMA_INCLUDE_DIRS        ... magma include directory
#   MAGMA_LIBRARIES           ... magma libraries
#
# The following variables will be checked by the function
#   MAGMA_USE_STATIC_LIBS     ... if true, only static libraries are found
#   MAGMA_ROOT                ... if set, the libraries are exclusively searched
#                                 under this path

#If environment variable MAGMA_ROOT is specified, it has same effect as MAGMA_ROOT
if( NOT MAGMA_ROOT AND NOT $ENV{MAGMA_ROOT} STREQUAL "" )
    set( MAGMA_ROOT $ENV{MAGMA_ROOT} )
    # set library directories
    set(MAGMA_LIBRARY_DIRS ${MAGMA_ROOT}/lib)
    # set include directories
    set(MAGMA_INCLUDE_DIRS ${MAGMA_ROOT}/include)
    # set libraries
    find_library(
        MAGMA_LIBRARIES
        NAMES "magma"
        PATHS ${MAGMA_ROOT}
        PATH_SUFFIXES "lib"
        NO_DEFAULT_PATH
    )
    set(MAGMA_FOUND TRUE)
else()
    set(MAGMA_FOUND FALSE)
endif()