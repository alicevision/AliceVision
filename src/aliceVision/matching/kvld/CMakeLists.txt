# Headers
set(kvld_files_headers
  kvld.h
  kvld_draw.h
  algorithm.h
)

# Sources
set(kvld_files_sources
  kvld.cpp
  algorithm.cpp
)

add_library(aliceVision_kvld
  ${kvld_files_headers}
  ${kvld_files_sources}
)

set_target_properties(aliceVision_kvld
  PROPERTIES SOVERSION ${ALICEVISION_VERSION_MAJOR}
  VERSION "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}"
)

target_link_libraries(aliceVision_kvld
  PUBLIC 
    aliceVision_image
    aliceVision_feature
    aliceVision_numeric
    aliceVision_system
)

set_property(TARGET aliceVision_kvld
  PROPERTY FOLDER AliceVision
)

install(TARGETS aliceVision_kvld
  DESTINATION lib
  EXPORT aliceVision-targets
)
