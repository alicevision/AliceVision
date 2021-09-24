# Headers
set(sfmDataIO_files_headers
  sfmDataIO.hpp
  bafIO.hpp
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
    Boost::filesystem
  PRIVATE_LINKS
    aliceVision_image
    Boost::filesystem
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
  alicevision_add_test(alembicIO_test.cpp NAME "sfmDataIO_alembic" LINKS aliceVision_sfmDataIO Alembic::Alembic)
endif()


alicevision_add_test(middlebury_test.cpp
        NAME "middlebury"
        LINKS aliceVision_sfmData
        aliceVision_numeric
        aliceVision_sfmDataIO
        )