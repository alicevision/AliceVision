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
    FlatBuffersIO.hpp
    MappedFile.hpp
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
    FlatBuffersIO.cpp
    MappedFile.cpp
)

if (ALICEVISION_HAVE_ALEMBIC)
    list(APPEND sfmDataIO_files_headers
        AlembicExporter.hpp
        AlembicImporter.hpp
        ExternalAlembicImporter.hpp
    )
    list(APPEND sfmDataIO_files_sources
        AlembicExporter.cpp
        AlembicImporter.cpp
        ExternalAlembicImporter.cpp
    )
endif()

list (APPEND FLATBUFFERS_SCHEMA_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/FlatBuffersIO/schemas/Vec.fbs
    ${CMAKE_CURRENT_SOURCE_DIR}/FlatBuffersIO/schemas/Rig.fbs
    ${CMAKE_CURRENT_SOURCE_DIR}/FlatBuffersIO/schemas/CameraPose.fbs
    ${CMAKE_CURRENT_SOURCE_DIR}/FlatBuffersIO/schemas/Observation.fbs
    ${CMAKE_CURRENT_SOURCE_DIR}/FlatBuffersIO/schemas/Landmark.fbs
    ${CMAKE_CURRENT_SOURCE_DIR}/FlatBuffersIO/schemas/Image.fbs
    ${CMAKE_CURRENT_SOURCE_DIR}/FlatBuffersIO/schemas/View.fbs
    ${CMAKE_CURRENT_SOURCE_DIR}/FlatBuffersIO/schemas/Intrinsic.fbs
    ${CMAKE_CURRENT_SOURCE_DIR}/FlatBuffersIO/schemas/Distortion.fbs
    ${CMAKE_CURRENT_SOURCE_DIR}/FlatBuffersIO/schemas/Undistortion.fbs
    ${CMAKE_CURRENT_SOURCE_DIR}/FlatBuffersIO/schemas/SurveyPoint.fbs
    ${CMAKE_CURRENT_SOURCE_DIR}/FlatBuffersIO/schemas/SfmData.fbs
    ${CMAKE_CURRENT_SOURCE_DIR}/FlatBuffersIO/schemas/Root.fbs
)

list (APPEND FLATBUFFERS_SCHEMA_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/FlatBuffersIO/schemas
)

build_flatbuffers(
    "${FLATBUFFERS_SCHEMA_FILES}"
    "${FLATBUFFERS_SCHEMA_INCLUDE_DIRS}"
    "fbschemas"
    ""
    "${generatedDir}"
    "${generatedDir}"
    "${generatedDir}"
)

set (FLATBUFFERS_FLATC_SCHEMA_EXTRA_ARGS "--gen-object-api")
add_library(FlatBuffersTarget INTERFACE)
target_include_directories(FlatBuffersTarget INTERFACE ${flatbuffers_SOURCE_DIR}/include)
add_dependencies(FlatBuffersTarget fbschemas)


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
        FlatBuffersTarget
)

if (ALICEVISION_HAVE_ALEMBIC)
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

alicevision_add_test(sfmDataIOCompatibilityJson_test.cpp
    NAME "sfmDataIOCompatibilityJson"
    LINKS aliceVision_sfmData
          aliceVision_sfmDataIO
          aliceVision_feature
          aliceVision_system
)

if (ALICEVISION_HAVE_ALEMBIC)
    alicevision_add_test(sfmDataIOCompatibilityAbc_test.cpp
        NAME "sfmDataIOCompatibilityAbc"
        LINKS aliceVision_sfmData
              aliceVision_sfmDataIO
              aliceVision_feature
              aliceVision_system
    )
    alicevision_add_test(alembicIO_test.cpp
        NAME "sfmDataIO_alembic"
        LINKS aliceVision_sfmDataIO
              aliceVision_sfm
              Alembic::Alembic
    )
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
    alicevision_swig_add_library(sfmDataIO
        SOURCES SfMDataIO.i
        PUBLIC_LINKS
            aliceVision_sfmDataIO
            ${Python3_LIBRARIES}
        PRIVATE_INCLUDE_DIRS
            ../include
            ${ALICEVISION_ROOT}/include
            ${Python3_INCLUDE_DIRS}
            ${Python3_NumPy_INCLUDE_DIRS}
    )
endif()
