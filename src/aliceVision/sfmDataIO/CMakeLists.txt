# Headers
set(sfmDataIO_files_headers
  sfmDataIO.hpp
  bafIO.hpp
  colmap.hpp
  gtIO.hpp
  jsonIO.hpp
  middlebury.hpp
  plyIO.hpp
  viewIO.hpp
  sceneSample.hpp
)

# Sources
set(sfmDataIO_files_sources
  sfmDataIO.cpp
  bafIO.cpp
  colmap.cpp
  gtIO.cpp
  jsonIO.cpp
  middlebury.cpp
  plyIO.cpp
  viewIO.cpp
  sceneSample.cpp
)

if(ALICEVISION_HAVE_ALEMBIC)
  list(APPEND sfmDataIO_files_headers
    AlembicExporter.hpp
    AlembicImporter.hpp
  )
  list(APPEND sfmDataIO_files_sources
    AlembicExporter.cpp
    AlembicImporter.cpp
  )
endif()

alicevision_add_library(aliceVision_sfmDataIO
  SOURCES ${sfmDataIO_files_headers} ${sfmDataIO_files_sources}
  PUBLIC_LINKS
    aliceVision_sfmData
    aliceVision_camera
  PRIVATE_LINKS
    assimp::assimp
    aliceVision_image
    Boost::regex
    Boost::boost
)


if(ALICEVISION_HAVE_ALEMBIC)
  target_link_libraries(aliceVision_sfmDataIO
    PRIVATE Alembic::Alembic
  )
  # There is a missing include dependency in Alembic cmake export.
  target_include_directories(aliceVision_sfmDataIO
    PRIVATE "${ILMBASE_INCLUDE_DIR}"
  )
endif()

# Unit tests

alicevision_add_test(sfmDataIO_test.cpp
  NAME "sfmDataIO"
  LINKS aliceVision_sfmData
        aliceVision_sfmDataIO
        aliceVision_feature
        aliceVision_system
)

alicevision_add_test(sfmDataIOCompatibility_test.cpp
  NAME "sfmDataIOCompatibility"
  LINKS aliceVision_sfmData
        aliceVision_sfmDataIO
        aliceVision_feature
        aliceVision_system
)

if(ALICEVISION_HAVE_ALEMBIC)
  alicevision_add_test(alembicIO_test.cpp NAME "sfmDataIO_alembic" LINKS aliceVision_sfmDataIO aliceVision_sfm Alembic::Alembic)
endif()


alicevision_add_test(middlebury_test.cpp
        NAME "sfmDataIO_middlebury"
        LINKS aliceVision_sfmData
        aliceVision_numeric
        aliceVision_sfmDataIO
        )

alicevision_add_test(colmap_test.cpp
        NAME "sfmDataIO_colmap"
        LINKS aliceVision_sfmData
        aliceVision_numeric
        aliceVision_sfmDataIO
        )


# SWIG Binding
if (ALICEVISION_BUILD_SWIG_BINDING)
    set(UseSWIG_TARGET_NAME_PREFERENCE STANDARD)
    set_property(SOURCE SfMDataIO.i PROPERTY CPLUSPLUS ON)
    set_property(SOURCE SfMDataIO.i PROPERTY SWIG_MODULE_NAME sfmDataIO)

    swig_add_library(sfmDataIO
        TYPE MODULE
        LANGUAGE python
        SOURCES SfMDataIO.i
    )

    if (CMAKE_SYSTEM_NAME MATCHES "Linux")
        list(APPEND SWIG_EXTRA_COMPILE_OPTIONS "-DSWIGWORDSIZE64")
    endif()

    set_property(
        TARGET sfmDataIO
        PROPERTY SWIG_COMPILE_OPTIONS -doxygen ${SWIG_EXTRA_COMPILE_OPTIONS}
    )

    target_include_directories(sfmDataIO
    PRIVATE
        ../include
        ${ALICEVISION_ROOT}/include
        ${Python3_INCLUDE_DIRS}
        ${Python3_NumPy_INCLUDE_DIRS}
    )
    set_property(
        TARGET sfmDataIO
        PROPERTY SWIG_USE_TARGET_INCLUDE_DIRECTORIES ON
    )
    set_property(
        TARGET sfmDataIO
        PROPERTY COMPILE_OPTIONS -std=c++17
    )

    target_link_libraries(sfmDataIO
    PUBLIC
        aliceVision_sfmDataIO
        ${Python3_LIBRARIES}
    )

    install(
    TARGETS
        sfmDataIO
    DESTINATION
        ${CMAKE_INSTALL_PREFIX}
    )
    install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/sfmDataIO.py
    DESTINATION
        ${CMAKE_INSTALL_PREFIX}
    )
endif()