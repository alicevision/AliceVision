#
# This module tries to find and setup an Alembic configuration for openMVG.
# You can help the search by providing several environment variable or cmake 
# variable:
# ALEMBIC_ROOT
# ALEMBIC_HDF5_ROOT
# ALEMBIC_ILMBASE_ROOT
# ALEMBIC_OPENEXR_ROOT
#
# HDF5 and ILMBASE should point to the root dir used to compile alembic
#
# This module provides variables prefixed with ABC
# It also sets ALEMBIC_FOUND if all the libraries and include dirs were found
#

MESSAGE(STATUS "Looking for Alembic. 1.5.8")

################################################################################
# IlmBase include dir for half and ilm libraries used in alembic
################################################################################

# Alembic includes half.h for a single function "half to float", this is unfortunate
FIND_PATH(ABC_HALF_INCLUDE_DIR half.h
    HINTS
        ${ALEMBIC_ROOT}
        ${ALEMBIC_ILMBASE_ROOT}
        ${ILMBASE_INCLUDE_DIR}
        $ENV{ALEMBIC_ROOT}
        $ENV{ALEMBIC_ILMBASE_ROOT}
        $ENV{ILMBASE_INCLUDE_DIR}
    PATH_SUFFIXES
        OpenEXR
	include
	include/OpenEXR
    )

FIND_LIBRARY(ABC_ILMBASE_IEX NAMES Iex Iex-2_2
    PATHS
        ${ALEMBIC_ILMBASE_ROOT}
        ${ILMBASE_LIBRARY_DIR}
        ${ALEMBIC_ROOT}
        $ENV{ILMBASE_LIBRARY_DIR}
        $ENV{ALEMBIC_ILMBASE_ROOT}
        $ENV{ALEMBIC_ROOT}
    PATH_SUFFIXES
        lib
	lib/static
	lib64
    )

FIND_LIBRARY(ABC_ILMBASE_IEXMATH NAMES IexMath IexMath-2_2
    PATHS
        ${ALEMBIC_ILMBASE_ROOT}
        ${ILMBASE_LIBRARY_DIR}
        ${ALEMBIC_ROOT}
        $ENV{ILMBASE_LIBRARY_DIR}
        $ENV{ALEMBIC_ILMBASE_ROOT}
        $ENV{ALEMBIC_ROOT}
    PATH_SUFFIXES
        lib
	lib/static
	lib64
    )
IF(ABC_ILMBASE_IEXMATH MATCHES ".*-NOTFOUND")
    # This library is optional, so ignore if not found.
    SET(ABC_ILMBASE_IEXMATH "")
ENDIF()

FIND_LIBRARY(ABC_ILMBASE_HALF NAMES Half
    PATHS
        ${ALEMBIC_ILMBASE_ROOT}
        ${ILMBASE_LIBRARY_DIR}
        ${ALEMBIC_ROOT}
        $ENV{ILMBASE_LIBRARY_DIR}
        $ENV{ALEMBIC_ILMBASE_ROOT}
        $ENV{ALEMBIC_ROOT}
    PATH_SUFFIXES
        lib
	lib/static
	lib64
    )

SET(ABC_ILMBASE_LIBS ${ABC_ILMBASE_IEX} ${ABC_ILMBASE_IEXMATH} ${ABC_ILMBASE_HALF})

# OpenEXR
FIND_LIBRARY(ABC_OPENEXR_LIBS IlmImf IlmImf-2_2
    PATHS 
        ${ALEMBIC_OPENEXR_ROOT}
        ${OPENEXR_LIBRARY_DIR}
        ${ALEMBIC_ROOT}
        $ENV{ALEMBIC_OPENEXR_ROOT}
        $ENV{OPENEXR_LIBRARY_DIR}
        $ENV{ALEMBIC_ROOT}
    PATH_SUFFIXES
        lib
	lib64
    )

################################################################################
# HDF5 libraries used in alembic
################################################################################

# FIXME: hdf5 should be handled by a specialized module
FIND_PATH(ABC_HDF5_LIBS_PATH NAMES hdf5
    PATHS 
        ${ALEMBIC_HDF5_ROOT}
        ${HDF5_LIBRARY_DIR}
        ${ALEMBIC_ROOT}
        $ENV{ALEMBIC_HDF5_ROOT}
        $ENV{HDF5_LIBRARY_DIR}
        $ENV{ALEMBIC_ROOT}
    PATH_SUFFIXES
        lib
	lib64
    )
FIND_LIBRARY(ABC_HDF5 hdf5 PATHS ${ABC_HDF5_LIBS_PATH})
FIND_LIBRARY(ABC_HDF5_HL hdf5_hl PATHS ${ABC_HDF5_LIBS_PATH})
SET(ABC_HDF5_LIBS ${ABC_HDF5} ${ABC_HDF5_HL})

################################################################################
# ALEMBIC include and library dir
################################################################################

FIND_PATH(ABC_INCLUDE_DIR Alembic/Abc/All.h
    PATHS
        $ENV{ALEMBIC_ROOT}/include
        $ENV{ALEMBIC_INCLUDE_DIR}
        ${ALEMBIC_ROOT}/include
        ${ALEMBIC_INCLUDE_DIR}
    PATH_SUFFIXES
        alembic
    )

# message(STATUS "ALEMBIC_ROOT: ${ALEMBIC_ROOT}")
# message(STATUS "ABC_INCLUDE_DIR: ${ABC_INCLUDE_DIR}")
# message(STATUS "ABC_OPENEXR_LIBS ${ABC_OPENEXR_LIBS}")

#
# We force the use of dynamic libraries as using the static ones had caused some 
# initialization problems.

# try to find the all-in-one library
message(STATUS "trying to found the all-in-one libAlembic.so")
FIND_LIBRARY(ABC_LIBRARY_ALLINONE Alembic
    PATHS
        ${ALEMBIC_LIBRARY_DIR}
        ${ALEMBIC_ROOT}
        $ENV{ALEMBIC_LIBRARY_DIR}
        $ENV{ALEMBIC_ROOT}
    PATH_SUFFIXES
        lib
	lib/static
	lib64
	static
    )
# message(STATUS "ABC_LIBRARY_ALLINONE ${ABC_LIBRARY_ALLINONE}")
if( EXISTS ${ABC_LIBRARY_ALLINONE})
    message(STATUS "found the all-in-one libAlembic: ${ABC_LIBRARY_ALLINONE}")
    SET(ABC_CORE_LIBS ${ABC_LIBRARY_ALLINONE})
    GET_FILENAME_COMPONENT(ABC_LIBRARY_DIR ${ABC_LIBRARY_ALLINONE} DIRECTORY)
else()
    message(STATUS "all-in-one not found, try finding individual ones --  ${ABC_LIBRARY_ALLINONE}")

    FIND_LIBRARY(ABC_LIBRARY AlembicAbc
        PATHS
            ${ALEMBIC_LIBRARY_DIR}
            ${ALEMBIC_ROOT}
            $ENV{ALEMBIC_ROOT}
            $ENV{ALEMBIC_LIBRARY_DIR}
	PATH_SUFFIXES
	    lib
	    lib/static
	    lib64
        )
    GET_FILENAME_COMPONENT(ABC_LIBRARY_DIR ${ABC_LIBRARY} DIRECTORY)

    FIND_LIBRARY(ABC_COLLECTION AlembicAbcCollection PATHS ${ABC_LIBRARY_DIR})
    FIND_LIBRARY(ABC_COREFACTORY AlembicAbcCoreFactory PATHS ${ABC_LIBRARY_DIR})
    FIND_LIBRARY(ABC_COREOGAWA AlembicAbcCoreOgawa PATHS ${ABC_LIBRARY_DIR})
    #FIND_LIBRARY(ABC_MATERIAL AlembicAbcMaterial PATHS ${ABC_LIBRARY_DIR})
    FIND_LIBRARY(ABC_OGAWA AlembicOgawa PATHS ${ABC_LIBRARY_DIR})
    #FIND_LIBRARY(ABC_OPENGL AlembicAbcOpenGL PATHS ${ABC_LIBRARY_DIR})
    #FIND_LIBRARY(ABC_WFOBJCONVERT AbcWFObjConvert PATHS ${ABC_LIBRARY_DIR})
    FIND_LIBRARY(ABC_COREABSTRACT AlembicAbcCoreAbstract PATHS ${ABC_LIBRARY_DIR})
    FIND_LIBRARY(ABC_COREHDF5 AlembicAbcCoreHDF5 PATHS ${ABC_LIBRARY_DIR})
    FIND_LIBRARY(ABC_GEOM AlembicAbcGeom PATHS ${ABC_LIBRARY_DIR})
    FIND_LIBRARY(ABC_UTIL AlembicUtil PATHS ${ABC_LIBRARY_DIR})
    SET(ABC_CORE_LIBS ${ABC_LIBRARY} 
                      ${ABC_GEOM} 
                      ${ABC} 
                      ${ABC_COREHDF5} 
                      ${ABC_COREABSTRACT} 
                      ${ABC_UTIL} 
                      ${ABC_COREFACTORY} 
                      ${ABC_OGAWA} 
                      ${ABC_COREOGAWA} 
                      ${ABC_COLLECTION})
endif()

SET(ABC_LIBRARIES ${ABC_CORE_LIBS} ${ABC_HDF5_LIBS} "-ldl" ${ABC_OPENEXR_LIBS} ${ABC_ILMBASE_LIBS})
SET(ABC_INCLUDE_DIR ${ABC_INCLUDE_DIR} ${ABC_HALF_INCLUDE_DIR})

message(STATUS "ABC_LIBRARIES: ${ABC_LIBRARIES}")
message(STATUS "ABC_INCLUDE_DIR: ${ABC_INCLUDE_DIR}")

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS("Alembic" DEFAULT_MSG ABC_LIBRARIES ABC_LIBRARY_DIR ABC_INCLUDE_DIR ABC_HDF5_LIBS)

if (ALEMBIC_FOUND)
    mark_as_advanced(ABC_LIBRARY_DIR ABC_HDF5_LIBS_PATH)
    message("Found Alembic - will build alembic exporter")
else()
    message("Alembic NOT FOUND")   
endif()

